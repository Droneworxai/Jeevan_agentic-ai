import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Ecoweeder - KML Generator & Viewer",
  description: "Advanced KML generation and viewing with AI navigation",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body className="font-sans">{children}</body>
    </html>
  );
}
