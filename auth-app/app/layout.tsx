import './globals.css'

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
      <body className="antialiased">{children}</body>
    </html>
  )
}
