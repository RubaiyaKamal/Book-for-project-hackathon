"use client";

import { useState, useEffect } from "react";
import BookSidebar from "@/components/BookSidebar";
import RelatedSidebar from "@/components/RelatedSidebar";
import Chatbot from "@/components/Chatbot";

interface BookContent {
    title: string;
    content: string;
    sections: string[];
}

export default function BookReaderPage() {
    const [activeId, setActiveId] = useState("intro");
    const [selectedText, setSelectedText] = useState("");
    const [currentContent, setCurrentContent] = useState<BookContent>({
        title: "Loading...",
        content: "",
        sections: [],
    });
    const [loading, setLoading] = useState(true);
    const [isPersonalized, setIsPersonalized] = useState(false);
    const [personalizing, setPersonalizing] = useState(false);
    const [originalContent, setOriginalContent] = useState("");

    useEffect(() => {
        async function loadContent() {
            setLoading(true);
            setIsPersonalized(false);
            try {
                const response = await fetch(`/api/book-content?id=${activeId}`);
                if (response.ok) {
                    const data = await response.json();
                    setCurrentContent(data);
                    setOriginalContent(data.content);
                } else {
                    setCurrentContent({
                        title: "Error",
                        content: "# Content Not Found\n\nThe requested content could not be loaded.",
                        sections: [],
                    });
                }
            } catch (error) {
                setCurrentContent({
                    title: "Error",
                    content: "# Error Loading Content\n\nPlease try again later.",
                    sections: [],
                });
            }
            setLoading(false);
        }

        loadContent();
    }, [activeId]);

    const handleTextSelection = () => {
        const selection = window.getSelection();
        const text = selection?.toString().trim() || "";
        if (text) {
            setSelectedText(text);
        }
    };

    const handlePersonalize = async () => {
        const token = localStorage.getItem("token");
        if (!token) {
            if (confirm("You need to log in to personalize content. Go to login page?")) {
                window.location.href = "/signin";
            }
            return;
        }

        setPersonalizing(true);
        try {
            const response = await fetch("/api/personalize", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${token}`,
                },
                body: JSON.stringify({
                    chapterId: activeId,
                    originalContent: originalContent,
                }),
            });

            if (response.ok) {
                const data = await response.json();
                setCurrentContent({
                    ...currentContent,
                    content: data.personalizedContent,
                });
                setIsPersonalized(true);
            } else {
                const error = await response.json();
                alert(error.error || "Failed to personalize content");
            }
        } catch (error) {
            console.error("Personalization error:", error);
            alert("Failed to personalize content. Please try again.");
        } finally {
            setPersonalizing(false);
        }
    };

    const handleResetContent = () => {
        setCurrentContent({
            ...currentContent,
            content: originalContent,
        });
        setIsPersonalized(false);
    };

    const renderMarkdown = (markdown: string) => {
        return markdown
            .split('\n')
            .map((line, index) => {
                // Skip frontmatter
                if (line.trim() === '---' || line.startsWith('sidebar_position:') || line.startsWith('title:') || line.startsWith('description:')) {
                    return '';
                }

                // Headers
                if (line.startsWith('# ')) {
                    const text = line.substring(2);
                    const id = text.toLowerCase().replace(/\s+/g, '-').replace(/[^\w-]/g, '');
                    return `<h1 key="${index}" id="${id}" class="text-4xl font-bold text-dark-brown dark:text-cream mb-6 mt-8">${text}</h1>`;
                }
                if (line.startsWith('## ')) {
                    const text = line.substring(3);
                    const id = text.toLowerCase().replace(/\s+/g, '-').replace(/[^\w-]/g, '');
                    return `<h2 key="${index}" id="${id}" class="text-3xl font-semibold text-dark-brown dark:text-cream mb-4 mt-6 border-b-2 border-goldenrod/30 pb-2">${text}</h2>`;
                }
                if (line.startsWith('### ')) {
                    const text = line.substring(4);
                    return `<h3 key="${index}" class="text-2xl font-semibold text-dark-brown dark:text-cream mb-3 mt-4">${text}</h3>`;
                }
                if (line.startsWith('#### ')) {
                    const text = line.substring(5);
                    return `<h4 key="${index}" class="text-xl font-semibold text-goldenrod dark:text-goldenrod mb-2 mt-3">${text}</h4>`;
                }

                // Bold text
                line = line.replace(/\*\*(.*?)\*\*/g, '<strong class="text-goldenrod font-semibold">$1</strong>');

                // Code blocks (simple inline code)
                line = line.replace(/`([^`]+)`/g, '<code class="bg-cream dark:bg-dark-brown/50 px-2 py-1 rounded text-sm font-mono text-mint">$1</code>');

                // Links
                line = line.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" class="text-goldenrod hover:text-goldenrod/80 underline">$1</a>');

                // List items
                if (line.match(/^(\s*)-\s+/)) {
                    const indent = line.match(/^(\s*)/)?.[1].length || 0;
                    const text = line.replace(/^(\s*)-\s+/, '');
                    return `<li key="${index}" class="text-dark-brown/80 dark:text-cream/80 mb-2 ml-${indent / 2 * 4}">${text}</li>`;
                }

                // Blockquotes
                if (line.startsWith('> ')) {
                    const text = line.substring(2);
                    return `<blockquote key="${index}" class="border-l-4 border-mint pl-4 italic text-dark-brown/70 dark:text-cream/70 my-4">${text}</blockquote>`;
                }

                // Regular paragraphs
                if (line.trim() && !line.startsWith('```') && !line.startsWith(':::')) {
                    return `<p key="${index}" class="text-dark-brown/80 dark:text-cream/80 mb-4 leading-relaxed">${line}</p>`;
                }

                return '';
            })
            .filter(line => line)
            .join('');
    };

    return (
        <div className="h-screen flex flex-col bg-warm-white dark:bg-dark-brown">
            {/* Header */}
            <header className="bg-cream dark:bg-dark-brown border-b border-dark-brown/10 dark:border-cream/10 px-6 py-4 flex-shrink-0">
                <div className="flex items-center justify-between">
                    <div className="flex items-center space-x-4">
                        <a href="/" className="text-2xl font-bold text-dark-brown dark:text-cream hover:text-goldenrod transition-colors">
                            Physical AI & Humanoid Robotics
                        </a>
                    </div>
                    <div className="flex items-center space-x-6">
                        <a
                            href="https://github.com/RubaiyaKamal/Book-for-project-hackathon"
                            target="_blank"
                            rel="noopener noreferrer"
                            className="flex items-center space-x-2 text-dark-brown dark:text-cream hover:text-goldenrod transition-colors"
                            aria-label="View on GitHub"
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                                <path d="M9 19c-5 1.5-5-2.5-7-3m14 6v-3.87a3.37 3.37 0 0 0-.94-2.61c3.14-.35 6.44-1.54 6.44-7A5.44 5.44 0 0 0 20 4.77 5.07 5.07 0 0 0 19.91 1S18.73.65 16 2.48a13.38 13.38 0 0 0-7 0C6.27.65 5.09 1 5.09 1A5.07 5.07 0 0 0 5 4.77a5.44 5.44 0 0 0-1.5 3.78c0 5.42 3.3 6.61 6.44 7A3.37 3.37 0 0 0 9 18.13V22"></path>
                            </svg>
                            <span className="font-medium">GitHub</span>
                        </a>
                    </div>
                </div>
            </header>

            {/* Main Content Area */}
            <div className="flex-1 flex overflow-hidden">
                {/* Left Sidebar - Navigation */}
                <aside className="w-80 flex-shrink-0 overflow-hidden">
                    <BookSidebar activeId={activeId} onSelect={setActiveId} />
                </aside>

                {/* Center - Book Content */}
                <main className="flex-1 overflow-y-auto px-8 py-6 bg-cream/30 dark:bg-dark-brown" onMouseUp={handleTextSelection}>
                    <article className="max-w-4xl mx-auto">
                        {loading ? (
                            <div className="flex items-center justify-center py-20">
                                <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-goldenrod"></div>
                            </div>
                        ) : (
                            <>
                                {/* Personalize Button */}
                                <div className="mb-6 flex items-center justify-between p-4 bg-goldenrod/10 dark:bg-goldenrod/5 rounded-lg border border-goldenrod/30">
                                    <div className="flex items-center space-x-3">
                                        <span className="text-2xl">✨</span>
                                        <div>
                                            <h3 className="text-sm font-semibold text-dark-brown dark:text-cream">
                                                {isPersonalized ? "Personalized for You" : "Personalize This Chapter"}
                                            </h3>
                                            <p className="text-xs text-dark-brown/70 dark:text-cream/70">
                                                {isPersonalized
                                                    ? "Content adapted to your experience level and interests"
                                                    : "Customize content based on your profile"}
                                            </p>
                                        </div>
                                    </div>
                                    <div className="flex space-x-2">
                                        {isPersonalized ? (
                                            <button
                                                onClick={handleResetContent}
                                                className="px-4 py-2 text-sm bg-cream dark:bg-dark-brown text-dark-brown dark:text-cream rounded-lg hover:bg-goldenrod/20 transition-colors border border-dark-brown/10 dark:border-cream/10"
                                            >
                                                Reset to Original
                                            </button>
                                        ) : (
                                            <button
                                                onClick={handlePersonalize}
                                                disabled={personalizing}
                                                className="px-4 py-2 text-sm bg-goldenrod text-dark-brown rounded-lg hover:bg-goldenrod/90 transition-colors font-semibold disabled:opacity-50 disabled:cursor-not-allowed"
                                            >
                                                {personalizing ? "Personalizing..." : "Personalize Content"}
                                            </button>
                                        )}
                                    </div>
                                </div>

                                <div
                                    className="prose prose-lg max-w-none"
                                    dangerouslySetInnerHTML={{
                                        __html: renderMarkdown(currentContent.content)
                                    }}
                                />

                                {/* Navigation Buttons */}
                                <div className="mt-12 pt-6 border-t border-dark-brown/10 dark:border-cream/10 flex justify-between">
                                    <button className="px-6 py-3 bg-cream dark:bg-dark-brown/50 text-dark-brown dark:text-cream rounded-lg hover:bg-goldenrod/20 transition-colors border border-dark-brown/10 dark:border-cream/10">
                                        ← Previous
                                    </button>
                                    <button className="px-6 py-3 bg-goldenrod text-dark-brown rounded-lg hover:bg-goldenrod/90 transition-colors font-semibold">
                                        Next →
                                    </button>
                                </div>
                            </>
                        )}
                    </article>
                </main>

                {/* Right Sidebar - Related Content */}
                <aside className="w-72 flex-shrink-0 overflow-hidden">
                    <RelatedSidebar sections={currentContent.sections} />
                </aside>
            </div>

            {/* Chatbot */}
            <Chatbot selectedText={selectedText} />
        </div>
    );
}
