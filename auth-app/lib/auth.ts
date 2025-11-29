import { betterAuth } from "better-auth";
import { neonAdapter } from "better-auth/adapters/neon";
import { Pool } from "@neondatabase/serverless";

const pool = new Pool({
    connectionString: process.env.NEON_DATABASE_URL,
});

export const auth = betterAuth({
    database: neonAdapter(pool),
    emailAndPassword: {
        enabled: true,
    },
    socialProviders: {
        google: {
            clientId: process.env.GOOGLE_CLIENT_ID!,
            clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
        },
        github: {
            clientId: process.env.GITHUB_CLIENT_ID!,
            clientSecret: process.env.GITHUB_CLIENT_SECRET!,
        },
    },
    session: {
        expiresIn: 60 * 60 * 24 * 7, // 7 days
        updateAge: 60 * 60 * 24, // 1 day
    },
    user: {
        // We'll store additional data in user_profiles table
    },
});

export type Session = typeof auth.$Infer.Session;
