'use client'

export default function GlobalError({
    error,
    reset,
}: {
    error: Error & { digest?: string }
    reset: () => void
}) {
    return (
        <html>
            <body>
                <div className="min-h-screen flex items-center justify-center bg-gradient-to-br from-gray-900 via-gray-800 to-gray-900">
                    <div className="text-center px-4 max-w-lg">
                        <h1 className="text-9xl font-bold text-transparent bg-clip-text bg-gradient-to-r from-red-400 to-orange-600">
                            Error
                        </h1>
                        <h2 className="text-3xl font-semibold text-white mt-4 mb-2">
                            Something Went Wrong
                        </h2>
                        <p className="text-gray-400 mb-8">
                            {error.message || 'An unexpected error occurred. Please try again.'}
                        </p>
                        <button
                            onClick={reset}
                            className="inline-block px-6 py-3 bg-gradient-to-r from-red-500 to-orange-600 text-white font-medium rounded-lg hover:from-red-600 hover:to-orange-700 transition-all duration-200 shadow-lg hover:shadow-xl"
                        >
                            Try Again
                        </button>
                    </div>
                </div>
            </body>
        </html>
    )
}
