import { auth } from "@/lib/auth";
import { toNextJsHandler } from "better-auth/next-js";

// Export GET and POST handlers for Next.js API routes
export const { GET, POST } = toNextJsHandler(auth);
