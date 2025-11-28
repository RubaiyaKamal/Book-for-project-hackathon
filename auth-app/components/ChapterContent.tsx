"use client";

import { useState } from "react";
import PersonalizeButton from "./PersonalizeButton";
import ReactMarkdown from "react-markdown";

interface ChapterContentProps {
    chapterId: string;
    originalContent: string;
    title: string;
}

export default function ChapterContent({
    chapterId,
    originalContent,
    title,
}: ChapterContentProps) {
    const [content, setContent] = useState(originalContent);
    const [profile, setProfile] = useState<any>(null);

    const handlePersonalized = (personalizedContent: string, userProfile: any) => {
        setContent(personalizedContent);
        setProfile(userProfile);
    };

    return (
        <div className="chapter-container">
            <h1 className="chapter-title">{title}</h1>

            {/* Personalization Button at the top */}
            <PersonalizeButton
                chapterId={chapterId}
                originalContent={originalContent}
                onPersonalized={handlePersonalized}
            />

            {/* Show profile info if personalized */}
            {profile && (
                <div className="profile-info">
                    <div className="profile-badge">
                        <span className="badge-label">Difficulty:</span>
                        <span className="badge-value">{profile.difficulty}</span>
                    </div>
                    <div className="profile-badge">
                        <span className="badge-label">Learning Style:</span>
                        <span className="badge-value">{profile.learningStyle}</span>
                    </div>
                    {profile.hasHardware && (
                        <div className="profile-badge">
                            <span className="badge-label">🤖 Hardware Mode:</span>
                            <span className="badge-value">Enabled</span>
                        </div>
                    )}
                </div>
            )}

            {/* Chapter content */}
            <div className={profile ? "personalized-content markdown-body" : "markdown-body"}>
                <ReactMarkdown>{content}</ReactMarkdown>
            </div>
        </div>
    );
}
