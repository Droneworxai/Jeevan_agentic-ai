import type { Metadata } from "next";
import "./globals.css";
import { validateEnv } from "@/lib/env";

export const metadata: Metadata = {
  title: "Ecoweeder - KML Generator & Viewer",
  description: "Advanced KML generation and viewing with AI navigation",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  // Validate environment variables on the server
  if (process.env.NODE_ENV !== 'test') {
    validateEnv();
  }

  return (
    <html lang="en" suppressHydrationWarning>
      <body className="font-sans" suppressHydrationWarning>{children}</body>
    </html>
  );
}
