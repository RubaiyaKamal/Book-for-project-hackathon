"use client";

import React, { createContext, useContext, useEffect, useState } from "react";
import { authClient } from "@/lib/auth-client";
import { useRouter } from "next/navigation";

interface User {
    email?: string;
    id: string;
    name?: string;
    image?: string | null;
}

interface AuthContextType {
    user: User | null;
    isAuthenticated: boolean;
    loading: boolean;
    login: () => void;
    logout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType>({
    user: null,
    isAuthenticated: false,
    loading: true,
    login: () => { },
    logout: async () => { },
});

export const useAuth = () => useContext(AuthContext);

export function AuthProvider({ children }: { children: React.ReactNode }) {
    const [mounted, setMounted] = useState(false);
    const router = useRouter();

    // Only run on client after mount
    useEffect(() => {
        setMounted(true);
    }, []);

    // Default values for SSR
    if (!mounted) {
        return (
            <AuthContext.Provider value={{
                user: null,
                isAuthenticated: false,
                loading: true,
                login: () => { },
                logout: async () => { },
            }}>
                {children}
            </AuthContext.Provider>
        );
    }

    // Client-side only
    return <AuthProviderClient>{children}</AuthProviderClient>;
}

function AuthProviderClient({ children }: { children: React.ReactNode }) {
    const { data: session, isPending } = authClient.useSession();
    const router = useRouter();

    const login = () => {
        router.push("/signin");
    };

    const logout = async () => {
        await authClient.signOut();
        router.push("/signin");
    };

    const user = session?.user ? {
        ...session.user,
        id: session.user.id
    } : null;

    return (
        <AuthContext.Provider value={{
            user,
            isAuthenticated: !!user,
            loading: isPending,
            login,
            logout
        }}>
            {children}
        </AuthContext.Provider>
    );
}
