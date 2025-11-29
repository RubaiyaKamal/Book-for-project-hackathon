import SigninForm from "@/components/auth/SigninForm";

export default function SigninPage() {
    return (
        <div className="min-h-screen bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
            <div className="text-center mb-8">
                <h1 className="text-3xl font-bold text-gray-900">Welcome Back</h1>
                <p className="mt-2 text-gray-600">
                    Continue your learning journey
                </p>
            </div>
            <SigninForm />
        </div>
    );
}
