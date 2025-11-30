import './globals.css'
import { AuthProvider } from '@/components/providers/AuthProvider'

export const metadata = {
  title: 'Physical AI Course - Authentication',
  description: 'Join the Physical AI & Humanoid Robotics course',
}

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <body className="antialiased">
        <AuthProvider>
          {children}
        </AuthProvider>
      </body>
    </html>
  )
}
