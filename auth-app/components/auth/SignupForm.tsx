"use client";

import { useState } from "react";
import { authClient } from "@/components/providers/AuthProvider";
import { useRouter } from "next/navigation";

interface SignupFormData {
    email: string;
    password: string;
    name: string;
}

interface BackgroundData {
    programming_experience: string;
    known_languages: string[];
    ml_experience: string;
    ros_experience: string;
    robotics_experience: string;
    electronics_knowledge: string;
    has_robot_hardware: boolean;
    hardware_platforms: string[];
    learning_style: string;
    preferred_pace: string;
    goals: string[];
}

export default function SignupForm() {
    const router = useRouter();
    const [step, setStep] = useState(1);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState("");

    // Step 1: Basic signup
    const [formData, setFormData] = useState<SignupFormData>({
        email: "",
        password: "",
        name: "",
    });

    // Step 2: Background questionnaire
    const [background, setBackground] = useState<BackgroundData>({
        programming_experience: "",
        known_languages: [],
        ml_experience: "",
        ros_experience: "",
        robotics_experience: "",
        electronics_knowledge: "",
        has_robot_hardware: false,
        hardware_platforms: [],
        learning_style: "",
        preferred_pace: "",
        goals: [],
    });

    const handleSignup = async (e: React.FormEvent) => {
        e.preventDefault();
        setLoading(true);
        setError("");

        try {
            const { data, error } = await authClient.signUp.email({
                email: formData.email,
                password: formData.password,
                name: formData.name,
            });

            if (error) {
                setError(error.message);
                setLoading(false);
                return;
            }

            // Move to background questionnaire
            setStep(2);
        } catch (err: any) {
            setError(err.message || "Signup failed");
        } finally {
            setLoading(false);
        }
    };

    const handleBackgroundSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        setLoading(true);

        try {
            const response = await fetch("/api/profile", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(background),
            });

            if (!response.ok) {
                throw new Error("Failed to save profile");
            }

            // Redirect to personalized dashboard
            router.push("/dashboard");
        } catch (err: any) {
            setError(err.message || "Failed to save background");
        } finally {
            setLoading(false);
        }
    };

    const toggleArrayItem = (array: string[], item: string) => {
        if (array.includes(item)) {
            return array.filter((i) => i !== item);
        }
        return [...array, item];
    };

    if (step === 1) {
        return (
            <div className="max-w-md mx-auto p-6 bg-cream dark:bg-dark-brown rounded-lg shadow-lg">
                <h2 className="text-2xl font-bold mb-6 text-dark-brown dark:text-cream">Create Your Account</h2>

                {error && (
                    <div className="mb-4 p-3 bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-200 rounded">
                        {error}
                    </div>
                )}

                <form onSubmit={handleSignup} className="space-y-4">
                    <div>
                        <label className="block text-sm font-medium mb-1 text-dark-brown dark:text-cream">Name</label>
                        <input
                            type="text"
                            value={formData.name}
                            onChange={(e) =>
                                setFormData({ ...formData, name: e.target.value })
                            }
                            className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg focus:ring-2 focus:ring-goldenrod bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream"
                            required
                        />
                    </div>

                    <div>
                        <label className="block text-sm font-medium mb-1 text-dark-brown dark:text-cream">Email</label>
                        <input
                            type="email"
                            value={formData.email}
                            onChange={(e) =>
                                setFormData({ ...formData, email: e.target.value })
                            }
                            className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg focus:ring-2 focus:ring-goldenrod bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream"
                            required
                        />
                    </div>

                    <div>
                        <label className="block text-sm font-medium mb-1 text-dark-brown dark:text-cream">Password</label>
                        <input
                            type="password"
                            value={formData.password}
                            onChange={(e) =>
                                setFormData({ ...formData, password: e.target.value })
                            }
                            className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg focus:ring-2 focus:ring-goldenrod bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream"
                            required
                            minLength={8}
                        />
                        <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
                            Minimum 8 characters
                        </p>
                    </div>

                    <button
                        type="submit"
                        disabled={loading}
                        className="w-full bg-goldenrod dark:bg-goldenrod text-dark-brown dark:text-dark-brown py-2 rounded-lg hover:bg-goldenrod/90 dark:hover:bg-goldenrod/80 disabled:bg-gray-400 dark:disabled:bg-gray-600 font-semibold"
                    >
                        {loading ? "Creating Account..." : "Continue"}
                    </button>
                </form>

                <div className="mt-6">
                    <div className="relative">
                        <div className="absolute inset-0 flex items-center">
                            <div className="w-full border-t border-gray-300 dark:border-dark-brown/30"></div>
                        </div>
                        <div className="relative flex justify-center text-sm">
                            <span className="px-2 bg-cream dark:bg-dark-brown text-gray-600 dark:text-cream/70">
                                Or continue with
                            </span>
                        </div>
                    </div>

                    <div className="mt-4 grid grid-cols-2 gap-3">
                        <button
                            onClick={() => authClient.signIn.social({ provider: "google" })}
                            className="flex items-center justify-center px-4 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg hover:bg-warm-white dark:hover:bg-dark-brown/30 text-dark-brown dark:text-cream"
                        >
                            Google
                        </button>

                        <button
                            onClick={() => authClient.signIn.social({ provider: "github" })}
                            className="flex items-center justify-center px-4 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg hover:bg-warm-white dark:hover:bg-dark-brown/30 text-dark-brown dark:text-cream"
                        >
                            GitHub
                        </button>
                    </div>
                </div>
            </div>
        );
    }

    // Step 2: Background Questionnaire
    return (
        <div className="max-w-2xl mx-auto p-6 bg-cream dark:bg-dark-brown rounded-lg shadow-lg">
            <h2 className="text-2xl font-bold mb-2 text-dark-brown dark:text-cream">Tell Us About Yourself</h2>
            <p className="text-dark-brown/70 dark:text-cream/70 mb-6">
                This helps us personalize your learning experience
            </p>

            <form onSubmit={handleBackgroundSubmit} className="space-y-6">
                {/* Software Background */}
                <div className="border-b border-gray-300 dark:border-dark-brown/30 pb-6">
                    <h3 className="text-lg font-semibold mb-4 text-dark-brown dark:text-cream">Software Background</h3>

                    <div className="space-y-4">
                        <div>
                            <label className="block text-sm font-medium mb-2 text-dark-brown dark:text-cream">
                                Programming Experience
                            </label>
                            <select
                                value={background.programming_experience}
                                onChange={(e) =>
                                    setBackground({
                                        ...background,
                                        programming_experience: e.target.value,
                                    })
                                }
                                className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream focus:ring-2 focus:ring-goldenrod"
                                required
                            >
                                <option value="">Select level</option>
                                <option value="beginner">Beginner (0-1 years)</option>
                                <option value="intermediate">Intermediate (1-3 years)</option>
                                <option value="advanced">Advanced (3-5 years)</option>
                                <option value="expert">Expert (5+ years)</option>
                            </select>
                        </div>

                        <div>
                            <label className="block text-sm font-medium mb-2 text-dark-brown dark:text-cream">
                                Programming Languages You Know
                            </label>
                            <div className="grid grid-cols-2 gap-2">
                                {["Python", "C++", "JavaScript", "Java", "C", "Rust"].map(
                                    (lang) => (
                                        <label key={lang} className="flex items-center text-dark-brown dark:text-cream">
                                            <input
                                                type="checkbox"
                                                checked={background.known_languages.includes(
                                                    lang.toLowerCase()
                                                )}
                                                onChange={() =>
                                                    setBackground({
                                                        ...background,
                                                        known_languages: toggleArrayItem(
                                                            background.known_languages,
                                                            lang.toLowerCase()
                                                        ),
                                                    })
                                                }
                                                className="mr-2"
                                            />
                                            {lang}
                                        </label>
                                    )
                                )}
                            </div>
                        </div>

                        <div>
                            <label className="block text-sm font-medium mb-2 text-dark-brown dark:text-cream">
                                Machine Learning Experience
                            </label>
                            <select
                                value={background.ml_experience}
                                onChange={(e) =>
                                    setBackground({ ...background, ml_experience: e.target.value })
                                }
                                className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream focus:ring-2 focus:ring-goldenrod"
                                required
                            >
                                <option value="">Select level</option>
                                <option value="none">No experience</option>
                                <option value="basic">Basic (completed courses)</option>
                                <option value="intermediate">
                                    Intermediate (built projects)
                                </option>
                                <option value="advanced">Advanced (professional work)</option>
                            </select>
                        </div>

                        <div>
                            <label className="block text-sm font-medium mb-2 text-dark-brown dark:text-cream">
                                ROS (Robot Operating System) Experience
                            </label>
                            <select
                                value={background.ros_experience}
                                onChange={(e) =>
                                    setBackground({ ...background, ros_experience: e.target.value })
                                }
                                className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream focus:ring-2 focus:ring-goldenrod"
                                required
                            >
                                <option value="">Select level</option>
                                <option value="none">No experience</option>
                                <option value="basic">Basic (tutorials)</option>
                                <option value="intermediate">Intermediate (projects)</option>
                                <option value="advanced">Advanced (professional)</option>
                            </select>
                        </div>
                    </div>
                </div>

                {/* Hardware Background */}
                <div className="border-b border-gray-200 dark:border-gray-700 pb-6">
                    <h3 className="text-lg font-semibold mb-4 text-gray-800 dark:text-gray-200">Hardware Background</h3>

                    <div className="space-y-4">
                        <div>
                            <label className="block text-sm font-medium mb-2 text-dark-brown dark:text-cream">
                                Robotics Experience
                            </label>
                            <select
                                value={background.robotics_experience}
                                onChange={(e) =>
                                    setBackground({
                                        ...background,
                                        robotics_experience: e.target.value,
                                    })
                                }
                                className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream focus:ring-2 focus:ring-goldenrod"
                                required
                            >
                                <option value="">Select level</option>
                                <option value="none">No experience</option>
                                <option value="hobbyist">Hobbyist</option>
                                <option value="professional">Professional</option>
                                <option value="expert">Expert/Researcher</option>
                            </select>
                        </div>

                        <div>
                            <label className="block text-sm font-medium mb-2 text-dark-brown dark:text-cream">
                                Electronics Knowledge
                            </label>
                            <select
                                value={background.electronics_knowledge}
                                onChange={(e) =>
                                    setBackground({
                                        ...background,
                                        electronics_knowledge: e.target.value,
                                    })
                                }
                                className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream focus:ring-2 focus:ring-goldenrod"
                                required
                            >
                                <option value="">Select level</option>
                                <option value="none">No knowledge</option>
                                <option value="basic">Basic (can read schematics)</option>
                                <option value="intermediate">
                                    Intermediate (can design circuits)
                                </option>
                                <option value="advanced">Advanced (PCB design)</option>
                            </select>
                        </div>

                        <div>
                            <label className="flex items-center">
                                <input
                                    type="checkbox"
                                    checked={background.has_robot_hardware}
                                    onChange={(e) =>
                                        setBackground({
                                            ...background,
                                            has_robot_hardware: e.target.checked,
                                        })
                                    }
                                    className="mr-2"
                                />
                                <span className="text-sm font-medium text-dark-brown dark:text-cream">
                                    I have access to robot hardware
                                </span>
                            </label>
                        </div>

                        {background.has_robot_hardware && (
                            <div>
                                <label className="block text-sm font-medium mb-2 text-dark-brown dark:text-cream">
                                    Hardware Platforms You Have
                                </label>
                                <div className="grid grid-cols-2 gap-2">
                                    {[
                                        "Raspberry Pi",
                                        "Arduino",
                                        "Jetson Nano",
                                        "Custom Robot",
                                        "Drone",
                                        "Robotic Arm",
                                    ].map((platform) => (
                                        <label key={platform} className="flex items-center text-dark-brown dark:text-cream">
                                            <input
                                                type="checkbox"
                                                checked={background.hardware_platforms.includes(
                                                    platform.toLowerCase().replace(" ", "-")
                                                )}
                                                onChange={() =>
                                                    setBackground({
                                                        ...background,
                                                        hardware_platforms: toggleArrayItem(
                                                            background.hardware_platforms,
                                                            platform.toLowerCase().replace(" ", "-")
                                                        ),
                                                    })
                                                }
                                                className="mr-2"
                                            />
                                            {platform}
                                        </label>
                                    ))}
                                </div>
                            </div>
                        )}
                    </div>
                </div>

                <button
                    type="submit"
                    disabled={loading}
                    className="w-full bg-mint dark:bg-mint text-white py-3 rounded-lg hover:bg-mint/90 dark:hover:bg-mint/80 font-semibold disabled:bg-gray-400 dark:disabled:bg-gray-600"
                >
                    {loading ? "Saving Profile..." : "Complete Setup"}
                </button>
            </form>
        </div>
    );
}
