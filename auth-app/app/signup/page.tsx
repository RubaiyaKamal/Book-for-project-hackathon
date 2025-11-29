import SignupForm from "@/components/auth/SignupForm";

export default function SignupPage() {
    return (
        <div className="min-h-screen bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
            <div className="text-center mb-8">
                <h1 className="text-3xl font-bold text-gray-900">
                    Join the Physical AI Course
                </h1>
                <p className="mt-2 text-gray-600">
                    Start your journey into humanoid robotics
                </p>
            </div>
            <SignupForm />
        </div>
    );
}
