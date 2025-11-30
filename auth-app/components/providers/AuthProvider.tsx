"use client";

import React, { createContext, useContext, useState, useEffect } from "react";
import { useRouter } from "next/navigation";

interface User {
    email: string;
    id: number;
    profile?: {
        software_experience: string;
        hardware_experience: string;
        interests: string[];
    };
}

interface AuthContextType {
    user: User | null;
    isAuthenticated: boolean;
    loading: boolean;
    login: (token: string) => Promise<string | null>;
    logout: () => void;
}

const AuthContext = createContext<AuthContextType>({
    user: null,
    isAuthenticated: false,
    loading: true,
    login: async () => "Not implemented",
    logout: () => { },
});

export const useAuth = () => useContext(AuthContext);

export function AuthProvider({ children }: { children: React.ReactNode }) {
    const [user, setUser] = useState<User | null>(null);
    const [loading, setLoading] = useState(true);
    const router = useRouter();

    useEffect(() => {
        const initAuth = async () => {
            const token = localStorage.getItem("token");
            if (token) {
                try {
                    const response = await fetch("http://127.0.0.1:8001/auth/me", {
                        headers: {
                            Authorization: `Bearer ${token}`,
                        },
                    });
                    if (response.ok) {
                        const userData = await response.json();
                        setUser(userData);
                    } else {
                        localStorage.removeItem("token");
                    }
                } catch (error) {
                    console.error("Auth check failed", error);
                }
            }
            setLoading(false);
        };

        initAuth();
    }, []);

    const login = async (token: string): Promise<string | null> => {
        localStorage.setItem("token", token);
        // Fetch user data immediately
        try {
            console.log("Fetching profile with token:", token.substring(0, 10) + "...");
            const response = await fetch("http://127.0.0.1:8001/auth/me", {
                headers: {
                    Authorization: `Bearer ${token}`,
                },
            });

            if (response.ok) {
                const userData = await response.json();
                setUser(userData);
                router.push("/dashboard");
                return null; // Success
            } else {
                const status = response.status;
                const text = await response.text();
                console.error("Login fetch failed:", status, text);
                return `Profile load failed: ${status} ${text}`;
            }
        } catch (error: any) {
            console.error("Login fetch failed", error);
            return `Network error: ${error.message}`;
        }
    };

    const logout = () => {
        localStorage.removeItem("token");
        setUser(null);
        router.push("/signin");
    };

    return (
        <AuthContext.Provider value={{ user, isAuthenticated: !!user, loading, login, logout }}>
            {children}
        </AuthContext.Provider>
    );
}
