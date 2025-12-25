"use client";

import { useEffect } from "react";
import { useRouter } from "next/navigation";
import { Button } from "@/components/ui/button";
import { Leaf, MapPin, Download, Bot } from "lucide-react";
import Link from "next/link";

export default function HomePage() {
  const router = useRouter();

  useEffect(() => {
    // Check if user is already logged in
    const user = localStorage.getItem("user");
    if (user) {
      router.push("/dashboard");
    }
  }, [router]);

  return (
    <div className="min-h-screen bg-gradient-to-br from-green-50 via-white to-blue-50">
      {/* Navigation */}
      <nav className="fixed top-0 left-0 right-0 bg-white/80 backdrop-blur-md border-b border-gray-200 z-50">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center h-16">
            <div className="flex items-center space-x-2">
              <div className="h-8 w-8 bg-green-600 rounded-full flex items-center justify-center">
                <Leaf className="h-5 w-5 text-white" />
              </div>
              <span className="text-xl font-bold text-gray-900">Ecoweeder</span>
            </div>
            <div className="flex items-center space-x-4">
              <Link href="/auth/login">
                <Button variant="ghost">Sign In</Button>
              </Link>
              <Link href="/auth/register">
                <Button className="bg-green-600 hover:bg-green-700">
                  Get Started
                </Button>
              </Link>
            </div>
          </div>
        </div>
      </nav>

      {/* Hero Section */}
      <section className="pt-32 pb-20 px-4">
        <div className="max-w-7xl mx-auto text-center">
          <div className="flex justify-center mb-6">
            <div className="h-20 w-20 bg-green-600 rounded-full flex items-center justify-center">
              <Leaf className="h-10 w-10 text-white" />
            </div>
          </div>
          <h1 className="text-5xl md:text-6xl font-bold text-gray-900 mb-6">
            Advanced KML Management
            <br />
            <span className="text-green-600">Powered by AI</span>
          </h1>
          <p className="text-xl text-gray-600 mb-8 max-w-2xl mx-auto">
            Generate, visualize, and navigate through KML files with our
            comprehensive platform. Featuring AI-powered navigation and
            intuitive map tools.
          </p>
          <div className="flex justify-center space-x-4">
            <Link href="/simulation">
              <Button
                size="lg"
                className="bg-blue-600 hover:bg-blue-700 text-lg px-8 shadow-lg shadow-blue-200"
              >
                Launch Simulation
              </Button>
            </Link>
            <Link href="/auth/register">
              <Button
                size="lg"
                className="bg-green-600 hover:bg-green-700 text-lg px-8"
              >
                Start Free Trial
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="py-20 px-4 bg-white">
        <div className="max-w-7xl mx-auto">
          <div className="text-center mb-16">
            <h2 className="text-4xl font-bold text-gray-900 mb-4">
              Powerful Features
            </h2>
            <p className="text-xl text-gray-600">
              Everything you need for KML management
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-8">
            <div className="p-6 bg-gradient-to-br from-green-50 to-green-100 rounded-xl">
              <div className="h-12 w-12 bg-green-600 rounded-lg flex items-center justify-center mb-4">
                <Download className="h-6 w-6 text-white" />
              </div>
              <h3 className="text-xl font-bold text-gray-900 mb-2">
                KML Generator
              </h3>
              <p className="text-gray-600">
                Draw polygons, lines, and markers on an interactive map. Export
                to KML format instantly.
              </p>
              <ul className="mt-4 space-y-2 text-sm text-gray-700">
                <li>✓ Multiple drawing tools</li>
                <li>✓ Real-time GeoJSON preview</li>
                <li>✓ One-click KML export</li>
              </ul>
            </div>

            <div className="p-6 bg-gradient-to-br from-blue-50 to-blue-100 rounded-xl">
              <div className="h-12 w-12 bg-blue-600 rounded-lg flex items-center justify-center mb-4">
                <MapPin className="h-6 w-6 text-white" />
              </div>
              <h3 className="text-xl font-bold text-gray-900 mb-2">
                KML Viewer
              </h3>
              <p className="text-gray-600">
                Load and visualize KML files on OpenStreetMap with detailed
                feature inspection.
              </p>
              <ul className="mt-4 space-y-2 text-sm text-gray-700">
                <li>✓ Fast KML parsing</li>
                <li>✓ Interactive markers</li>
                <li>✓ Feature details panel</li>
              </ul>
            </div>

            <div className="p-6 bg-gradient-to-br from-purple-50 to-purple-100 rounded-xl">
              <div className="h-12 w-12 bg-purple-600 rounded-lg flex items-center justify-center mb-4">
                <Bot className="h-6 w-6 text-white" />
              </div>
              <h3 className="text-xl font-bold text-gray-900 mb-2">
                AI Navigation
              </h3>
              <p className="text-gray-600">
                Let AI guide vehicles through your waypoints with intelligent
                pathfinding.
              </p>
              <ul className="mt-4 space-y-2 text-sm text-gray-700">
                <li>✓ Gemini AI powered</li>
                <li>✓ Auto-navigation</li>
                <li>✓ Real-time tracking</li>
              </ul>
            </div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="py-20 px-4 bg-gradient-to-r from-green-600 to-blue-600">
        <div className="max-w-4xl mx-auto text-center">
          <h2 className="text-4xl font-bold text-white mb-4">
            Ready to Get Started?
          </h2>
          <p className="text-xl text-green-50 mb-8">
            Join thousands of users managing their KML files with Ecoweeder
          </p>
          <Link href="/auth/register">
            <Button
              size="lg"
              className="bg-white text-green-600 hover:bg-gray-100 text-lg px-8"
            >
              Create Free Account
            </Button>
          </Link>
        </div>
      </section>

      {/* Footer */}
      <footer className="py-12 px-4 bg-gray-900 text-white">
        <div className="max-w-7xl mx-auto text-center">
          <div className="flex items-center justify-center space-x-2 mb-4">
            <div className="h-8 w-8 bg-green-600 rounded-full flex items-center justify-center">
              <Leaf className="h-5 w-5 text-white" />
            </div>
            <span className="text-xl font-bold">Ecoweeder</span>
          </div>
          <p className="text-gray-400 mb-4">Advanced KML Management Platform</p>
          <p className="text-sm text-gray-500">
            © 2024 Ecoweeder. All rights reserved.
          </p>
        </div>
      </footer>
    </div>
  );
}
