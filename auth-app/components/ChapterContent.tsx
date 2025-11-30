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
    const [urduContent, setUrduContent] = useState<string | null>(null);
    const [isTranslated, setIsTranslated] = useState(false);
    const [profile, setProfile] = useState<any>(null);

    const handlePersonalized = (personalizedContent: string, userProfile: any) => {
        setContent(personalizedContent);
        setProfile(userProfile);
        setIsTranslated(false); // Reset translation when personalizing
    };

    const handleTranslate = () => {
        if (!isTranslated) {
            // Placeholder for translation logic
            setUrduContent("یہ اردو میں ترجمہ شدہ مواد ہے۔"); // Dummy Urdu content
        }
        setIsTranslated(!isTranslated);
    };

    return (
        <div className="chapter-container">
            <h1 className="chapter-title">{title}</h1>

            <div className="button-group">
                {/* Personalization Button */}
                <PersonalizeButton
                    chapterId={chapterId}
                    originalContent={originalContent}
                    onPersonalized={handlePersonalized}
                />
                {/* Translate Button */}
                <button onClick={handleTranslate} className="translate-button">
                    {isTranslated ? "Show Original" : "Translate to Urdu"}
                </button>
            </div>

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
                <ReactMarkdown>{isTranslated && urduContent ? urduContent : content}</ReactMarkdown>
            </div>
        </div>
    );
}

// Add some basic styling for the button group and translate button
const styles = `
.button-group {
    display: flex;
    gap: 10px; /* Space between buttons */
    margin-bottom: 20px;
}

.translate-button {
    background-color: #4CAF50; /* Green */
    border: none;
    color: white;
    padding: 10px 20px;
    text-align: center;
    text-decoration: none;
    display: inline-block;
    font-size: 16px;
    margin: 4px 2px;
    cursor: pointer;
    border-radius: 8px;
}

.translate-button:hover {
    background-color: #45a049;
}
`;

// Inject styles into the head (for quick demo purposes, usually handled by CSS files)
if (typeof window !== 'undefined') {
  const styleSheet = document.createElement("style");
  styleSheet.type = "text/css";
  styleSheet.innerText = styles;
  document.head.appendChild(styleSheet);
}

