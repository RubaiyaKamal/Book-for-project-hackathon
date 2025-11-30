import { NextRequest, NextResponse } from "next/server";
import { PersonalizationEngine } from "@/lib/personalization";

export async function POST(req: NextRequest) {
    try {
        // Get JWT token from Authorization header
        const authHeader = req.headers.get("authorization");
        if (!authHeader || !authHeader.startsWith("Bearer ")) {
            return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
        }

        const token = authHeader.substring(7);

        // Fetch user profile from FastAPI
        const userResponse = await fetch("http://127.0.0.1:8001/auth/me", {
            headers: {
                Authorization: `Bearer ${token}`,
            },
        });

        if (!userResponse.ok) {
            return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
        }

        const user = await userResponse.json();
        const profile = user.profile;

        if (!profile) {
            return NextResponse.json(
                { error: "Profile not found. Please complete onboarding." },
                { status: 404 }
            );
        }

        const { chapterId, originalContent } = await req.json();

        if (!chapterId || !originalContent) {
            return NextResponse.json(
                { error: "Missing chapterId or originalContent" },
                { status: 400 }
            );
        }

        // Generate personalized content using OpenAI
        const personalizedContent = await generatePersonalizedContent(
            originalContent,
            profile
        );

        return NextResponse.json({
            success: true,
            personalizedContent,
            profile: {
                difficulty: PersonalizationEngine.getContentDifficulty(profile),
                interests: profile.interests || [],
                hardwareExperience: profile.hardware_experience,
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
    profile: any
): Promise<string> {
    const prompt = PersonalizationEngine.getPersonalizationPrompt(profile, originalContent);

    // Call OpenAI to personalize content
    const response = await fetch("https://api.openai.com/v1/chat/completions", {
        method: "POST",
        headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${process.env.OPENAI_API_KEY}`,
        },
        body: JSON.stringify({
            model: "gpt-4o-mini",
            messages: [
                {
                    role: "user",
                    content: prompt,
                },
            ],
            temperature: 0.7,
            max_tokens: 4000,
        }),
    });

    if (!response.ok) {
        const errorDetails = await response.text();
        throw new Error(`OpenAI API error: ${response.statusText}. Details: ${errorDetails}`);
    }

    const data = await response.json();
    return data.choices[0].message.content;
}
