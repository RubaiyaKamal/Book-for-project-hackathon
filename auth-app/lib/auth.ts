import { betterAuth } from "better-auth";
import { Pool } from "@neondatabase/serverless";

const pool = new Pool({
    connectionString: process.env.NEON_DATABASE_URL,
});

// Auth configuration
export const auth = betterAuth({
    database: pool,
    baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3000",
    trustedOrigins: [
        "http://localhost:3000",
        "https://book-for-project-hackathon-1.vercel.app",
        process.env.BETTER_AUTH_URL || "",
        process.env.NEXT_PUBLIC_APP_URL || "",
    ].filter(Boolean),
    emailAndPassword: {
        enabled: true,
    },
    socialProviders: {
        google: {
            clientId: process.env.GOOGLE_CLIENT_ID || "placeholder",
            clientSecret: process.env.GOOGLE_CLIENT_SECRET || "placeholder",
        },
        github: {
            clientId: process.env.GITHUB_CLIENT_ID || "placeholder",
            clientSecret: process.env.GITHUB_CLIENT_SECRET || "placeholder",
        },
    },
    session: {
        expiresIn: 60 * 60 * 24 * 7,
        updateAge: 60 * 60 * 24,
    },
});

export type Session = typeof auth.$Infer.Session;
