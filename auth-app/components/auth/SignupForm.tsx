"use client";

import { useState } from "react";
import { useAuth } from "@/components/providers/AuthProvider";
import { useRouter } from "next/navigation";

interface SignupFormData {
    email: string;
    password: string;
    name: string;
}

interface BackgroundData {
    software_experience: string;
    hardware_experience: string;
    interests: string[];
}

export default function SignupForm() {
    const router = useRouter();
    const { login } = useAuth();
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
        software_experience: "",
        hardware_experience: "",
        interests: [],
    });

    const handleBasicSubmit = (e: React.FormEvent) => {
        e.preventDefault();
        setStep(2);
    };

    const handleFinalSubmit = async (e: React.FormEvent) => {
        console.log("handleFinalSubmit triggered");
        e.preventDefault();
        setLoading(true);
        setError("");

        try {
            const payload = {
                email: formData.email,
                password: formData.password,
                software_experience: background.software_experience,
                hardware_experience: background.hardware_experience,
                interests: background.interests,
            };

            const response = await fetch("http://127.0.0.1:8001/auth/signup", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(payload),
            });
            console.log("Signup API response status:", response.status);

            if (!response.ok) {
                const data = await response.json();
                console.error("Signup API error data:", data);
                throw new Error(data.detail || "Signup failed");
            }
            console.log("Signup API success.");

            // Auto login after signup
            const loginResponse = await fetch("http://127.0.0.1:8001/auth/login", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({
                    email: formData.email,
                    password: formData.password,
                }),
            });
            console.log("Auto-login API response status:", loginResponse.status);

            if (loginResponse.ok) {
                const loginData = await loginResponse.json();
                console.log("Auto-login API success data:", loginData);
                const errorMsg = await login(loginData.access_token);
                if (errorMsg) {
                    console.error("Auto-login failed:", errorMsg);
                    router.push("/signin");
                } else {
                    router.push("/book"); // Redirect to book page on successful auto-login
                }
            } else {
                console.error("Auto-login API failed.");
                router.push("/signin");
            }
        } catch (err: any) {
            setError(err.message || "Signup failed");
        } finally {
            setLoading(false);
        }
    };

    const toggleInterest = (interest: string) => {
        if (background.interests.includes(interest)) {
            setBackground({
                ...background,
                interests: background.interests.filter((i) => i !== interest),
            });
        } else {
            setBackground({
                ...background,
                interests: [...background.interests, interest],
            });
        }
    };

    if (step === 1) {
        return (
            <div className="max-w-md mx-auto p-6 bg-cream dark:bg-dark-brown rounded-lg shadow-lg">
                <h2 className="text-2xl font-bold mb-6 text-dark-brown dark:text-cream">Create Your Account</h2>

                <form onSubmit={handleBasicSubmit} className="space-y-4">
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
                    </div>

                    <button
                        type="submit"
                        className="w-full bg-goldenrod dark:bg-goldenrod text-dark-brown dark:text-dark-brown py-2 rounded-lg hover:bg-goldenrod/90 dark:hover:bg-goldenrod/80 font-semibold"
                    >
                        Continue
                    </button>
                </form>
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

            {error && (
                <div className="mb-4 p-3 bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-200 rounded">
                    {error}
                </div>
            )}

            <form onSubmit={handleFinalSubmit} className="space-y-6">
                {/* Software Background */}
                <div className="border-b border-gray-300 dark:border-dark-brown/30 pb-6">
                    <h3 className="text-lg font-semibold mb-4 text-dark-brown dark:text-cream">Software Experience</h3>
                    <select
                        value={background.software_experience}
                        onChange={(e) =>
                            setBackground({
                                ...background,
                                software_experience: e.target.value,
                            })
                        }
                        className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream focus:ring-2 focus:ring-goldenrod"
                        required
                    >
                        <option value="">Select level</option>
                        <option value="beginner">Beginner</option>
                        <option value="intermediate">Intermediate</option>
                        <option value="expert">Expert</option>
                    </select>
                </div>

                {/* Hardware Background */}
                <div className="border-b border-gray-300 dark:border-dark-brown/30 pb-6">
                    <h3 className="text-lg font-semibold mb-4 text-dark-brown dark:text-cream">Hardware Experience</h3>
                    <select
                        value={background.hardware_experience}
                        onChange={(e) =>
                            setBackground({
                                ...background,
                                hardware_experience: e.target.value,
                            })
                        }
                        className="w-full px-3 py-2 border border-gray-300 dark:border-dark-brown/30 rounded-lg bg-white dark:bg-dark-brown/50 text-dark-brown dark:text-cream focus:ring-2 focus:ring-goldenrod"
                        required
                    >
                        <option value="">Select level</option>
                        <option value="none">None</option>
                        <option value="arduino">Arduino</option>
                        <option value="raspberry_pi">Raspberry Pi</option>
                        <option value="professional">Professional Robotics</option>
                    </select>
                </div>

                {/* Interests */}
                <div>
                    <h3 className="text-lg font-semibold mb-4 text-dark-brown dark:text-cream">Interests</h3>
                    <div className="grid grid-cols-2 gap-2">
                        {["ROS", "Computer Vision", "Reinforcement Learning", "Simulations", "Hardware Design"].map((interest) => (
                            <label key={interest} className="flex items-center text-dark-brown dark:text-cream">
                                <input
                                    type="checkbox"
                                    checked={background.interests.includes(interest)}
                                    onChange={() => toggleInterest(interest)}
                                    className="mr-2"
                                />
                                {interest}
                            </label>
                        ))}
                    </div>
                </div>

                <div className="flex gap-4">
                    <button
                        type="button"
                        onClick={() => setStep(1)}
                        className="w-1/3 px-4 py-3 border border-gray-300 dark:border-dark-brown/30 rounded-lg hover:bg-gray-50 dark:hover:bg-dark-brown/30 text-dark-brown dark:text-cream"
                    >
                        Back
                    </button>
                    <button
                        type="submit"
                        disabled={loading}
                        className="w-2/3 bg-mint dark:bg-mint text-white py-3 rounded-lg hover:bg-mint/90 dark:hover:bg-mint/80 font-semibold disabled:bg-gray-400 dark:disabled:bg-gray-600"
                    >
                        {loading ? "Creating Account..." : "Complete Setup"}
                    </button>
                </div>
            </form>
        </div>
    );
}
