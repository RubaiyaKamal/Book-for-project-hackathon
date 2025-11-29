"use client";

import { createAuthClient } from "better-auth/react";
import { ReactNode } from "react";

export const authClient = createAuthClient({
    baseURL: process.env.NEXT_PUBLIC_APP_URL || "http://localhost:3000",
});

export function AuthProvider({ children }: { children: ReactNode }) {
    return <>{children}</>;
}
