import { betterAuth } from "better-auth";
import { Pool } from "@neondatabase/serverless";

// Create Neon database pool
const pool = new Pool({
    connectionString: process.env.NEON_DATABASE_URL!,
});

// Better Auth configuration
export const auth = betterAuth({
    database: {
        provider: "postgres",
        url: process.env.NEON_DATABASE_URL!,
    },
    emailAndPassword: {
        enabled: true,
        requireEmailVerification: false, // Set to true in production
        minPasswordLength: 8,
    },
    socialProviders: {
        google: {
            clientId: process.env.GOOGLE_CLIENT_ID || "",
            clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
            enabled: !!process.env.GOOGLE_CLIENT_ID,
        },
        github: {
            clientId: process.env.GITHUB_CLIENT_ID || "",
            clientSecret: process.env.GITHUB_CLIENT_SECRET || "",
            enabled: !!process.env.GITHUB_CLIENT_ID,
        },
    },
    session: {
        expiresIn: 60 * 60 * 24 * 7, // 7 days
        updateAge: 60 * 60 * 24, // Update session every 24 hours
        cookieCache: {
            enabled: true,
            maxAge: 5 * 60, // 5 minutes
        },
    },
    advanced: {
        cookiePrefix: "book-auth",
        crossSubDomainCookies: {
            enabled: false,
        },
    },
});

export type Session = typeof auth.$Infer.Session;
export type User = typeof auth.$Infer.Session.user;
