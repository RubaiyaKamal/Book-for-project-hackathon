"use client";

import { useState } from "react";
import { authClient } from "@/components/providers/AuthProvider";
import { useRouter } from "next/navigation";
import Link from "next/link";

export default function SigninForm() {
    const router = useRouter();
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState("");
    const [formData, setFormData] = useState({
        email: "",
        password: "",
    });

    const handleSignin = async (e: React.FormEvent) => {
        e.preventDefault();
        setLoading(true);
        setError("");

        try {
            const { data, error } = await authClient.signIn.email({
                email: formData.email,
                password: formData.password,
            });

            if (error) {
                setError(error.message);
                setLoading(false);
                return;
            }

            router.push("/dashboard");
        } catch (err: any) {
            setError(err.message || "Signin failed");
            setLoading(false);
        }
    };

    return (
        <div className="max-w-md mx-auto p-6 bg-white rounded-lg shadow-lg">
            <h2 className="text-2xl font-bold mb-6">Welcome Back</h2>

            {error && (
                <div className="mb-4 p-3 bg-red-100 text-red-700 rounded">{error}</div>
            )}

            <form onSubmit={handleSignin} className="space-y-4">
                <div>
                    <label className="block text-sm font-medium mb-1">Email</label>
                    <input
                        type="email"
                        value={formData.email}
                        onChange={(e) =>
                            setFormData({ ...formData, email: e.target.value })
                        }
                        className="w-full px-3 py-2 border rounded-lg focus:ring-2 focus:ring-blue-500"
                        required
                    />
                </div>

                <div>
                    <label className="block text-sm font-medium mb-1">Password</label>
                    <input
                        type="password"
                        value={formData.password}
                        onChange={(e) =>
                            setFormData({ ...formData, password: e.target.value })
                        }
                        className="w-full px-3 py-2 border rounded-lg focus:ring-2 focus:ring-blue-500"
                        required
                    />
                </div>

                <button
                    type="submit"
                    disabled={loading}
                    className="w-full bg-blue-600 text-white py-2 rounded-lg hover:bg-blue-700 disabled:bg-gray-400"
                >
                    {loading ? "Signing In..." : "Sign In"}
                </button>
            </form>

            <div className="mt-6">
                <div className="relative">
                    <div className="absolute inset-0 flex items-center">
                        <div className="w-full border-t border-gray-300"></div>
                    </div>
                    <div className="relative flex justify-center text-sm">
                        <span className="px-2 bg-white text-gray-500">
                            Or continue with
                        </span>
                    </div>
                </div>

                <div className="mt-4 grid grid-cols-2 gap-3">
                    <button
                        onClick={() => authClient.signIn.social({ provider: "google" })}
                        className="flex items-center justify-center px-4 py-2 border rounded-lg hover:bg-gray-50"
                    >
                        Google
                    </button>

                    <button
                        onClick={() => authClient.signIn.social({ provider: "github" })}
                        className="flex items-center justify-center px-4 py-2 border rounded-lg hover:bg-gray-50"
                    >
                        GitHub
                    </button>
                </div>
            </div>

            <p className="mt-4 text-center text-sm text-gray-600">
                Don't have an account?{" "}
                <Link href="/signup" className="text-blue-600 hover:underline">
                    Sign up
                </Link>
            </p>
        </div>
    );
}
