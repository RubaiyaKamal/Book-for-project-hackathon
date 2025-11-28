import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { Pool } from "@neondatabase/serverless";
import { PersonalizationEngine } from "@/lib/personalization";

const pool = new Pool({
    connectionString: process.env.NEON_DATABASE_URL!,
});

export async function POST(req: NextRequest) {
    try {
        const session = await auth.api.getSession({
            headers: req.headers,
        });

        if (!session) {
            return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
        }

        const { chapterId, originalContent } = await req.json();

        if (!chapterId || !originalContent) {
            return NextResponse.json(
                { error: "Missing chapterId or originalContent" },
                { status: 400 }
            );
        }

        // Get user profile
        const profileResult = await pool.query(
            "SELECT * FROM user_profiles WHERE user_id = $1",
            [session.user.id]
        );

        if (profileResult.rows.length === 0) {
            return NextResponse.json(
                { error: "Profile not found. Please complete onboarding." },
                { status: 404 }
            );
        }

        const profile = profileResult.rows[0];

        // Generate personalized content
        const personalizedContent = await generatePersonalizedContent(
            originalContent,
            profile,
            chapterId
        );

        // Track that user personalized this chapter
        await pool.query(
            `INSERT INTO user_progress (user_id, chapter_id, started_at, last_accessed)
       VALUES ($1, $2, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
       ON CONFLICT (user_id, chapter_id)
       DO UPDATE SET last_accessed = CURRENT_TIMESTAMP`,
            [session.user.id, chapterId]
        );

        return NextResponse.json({
            success: true,
            personalizedContent,
            profile: {
                difficulty: PersonalizationEngine.getContentDifficulty(profile),
                learningStyle: profile.learning_style,
                hasHardware: profile.has_robot_hardware,
            },
        });
    } catch (error) {
        console.error("Personalization error:", error);
        return NextResponse.json(
            { error: "Failed to personalize content" },
            { status: 500 }
        );
    }
}

async function generatePersonalizedContent(
    originalContent: string,
    profile: any,
    chapterId: string
): Promise<string> {
    const difficulty = PersonalizationEngine.getContentDifficulty(profile);
    const systemPrompt = PersonalizationEngine.getChatbotSystemPrompt(profile);

    // Call OpenAI to personalize content
    const response = await fetch("https://api.openai.com/v1/chat/completions", {
        method: "POST",
        headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${process.env.OPENAI_API_KEY}`,
        },
        body: JSON.stringify({
            model: "gpt-4-turbo-preview",
            messages: [
                {
                    role: "system",
                    content: `${systemPrompt}

You are personalizing textbook chapter content. Adapt the content to match the user's profile while maintaining all key information.

Instructions:
- Adjust complexity to ${difficulty} level
- Emphasize ${profile.learning_style} learning approach
- ${profile.has_robot_hardware ? `Include practical examples for ${profile.hardware_platforms.join(", ")}` : "Focus on simulation and theory"}
- Keep the same structure and headings
- Maintain all code examples but adjust comments and explanations
- Add personalized tips in this format: "💡 Personalized Tip: [tip]"
- Make explanations match the user's experience level`,
                },
                {
                    role: "user",
                    content: `Personalize this chapter content:\n\n${originalContent.substring(0, 8000)}`,
                },
            ],
            temperature: 0.7,
            max_tokens: 4000,
        }),
    });

    if (!response.ok) {
        throw new Error(`OpenAI API error: ${response.statusText}`);
    }

    const data = await response.json();
    return data.choices[0].message.content;
}
