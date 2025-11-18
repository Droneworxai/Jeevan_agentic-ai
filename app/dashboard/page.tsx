"use client";

import { useRouter } from "next/navigation";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Download, MapPin, Bot, ArrowRight } from "lucide-react";

export default function DashboardPage() {
  const router = useRouter();

  const features = [
    {
      title: "KML Generator",
      description: "Create custom KML files by drawing on an interactive map",
      icon: Download,
      href: "/dashboard/kml-generator",
      color: "bg-green-100 text-green-600",
      buttonColor: "bg-green-600 hover:bg-green-700",
    },
    {
      title: "KML Viewer",
      description: "Load and visualize your KML files on OpenStreetMap",
      icon: MapPin,
      href: "/dashboard/kml-viewer",
      color: "bg-blue-100 text-blue-600",
      buttonColor: "bg-blue-600 hover:bg-blue-700",
    },
    {
      title: "AI Navigation",
      description: "AI-powered vehicle navigation through waypoints",
      icon: Bot,
      href: "/dashboard/ai-agent",
      color: "bg-purple-100 text-purple-600",
      buttonColor: "bg-purple-600 hover:bg-purple-700",
    },
  ];

  return (
    <div className="p-6">
      <div className="max-w-6xl mx-auto">
        <div className="mb-8">
          <h1 className="text-3xl font-bold text-gray-900 mb-2">
            Welcome to Ecoweeder Dashboard
          </h1>
          <p className="text-gray-600">
            Choose a tool below to get started with KML generation, viewing, or
            AI navigation
          </p>
        </div>

        <div className="grid gap-6 md:grid-cols-3 mb-8">
          {features.map((feature) => {
            const Icon = feature.icon;
            return (
              <Card
                key={feature.href}
                className="hover:shadow-lg transition-shadow"
              >
                <CardHeader>
                  <div
                    className={`h-12 w-12 rounded-lg flex items-center justify-center mb-3 ${feature.color}`}
                  >
                    <Icon className="h-6 w-6" />
                  </div>
                  <CardTitle className="text-xl">{feature.title}</CardTitle>
                  <CardDescription>{feature.description}</CardDescription>
                </CardHeader>
                <CardContent>
                  <Button
                    className={`w-full ${feature.buttonColor}`}
                    onClick={() => router.push(feature.href)}
                  >
                    Open Tool
                    <ArrowRight className="ml-2 h-4 w-4" />
                  </Button>
                </CardContent>
              </Card>
            );
          })}
        </div>

        <Card className="bg-gradient-to-r from-green-50 to-blue-50">
          <CardHeader>
            <CardTitle>Quick Start Guide</CardTitle>
            <CardDescription>
              Get started with Ecoweeder in 3 easy steps
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-4">
              <div className="flex items-start space-x-3">
                <div className="flex-shrink-0 h-8 w-8 bg-green-600 text-white rounded-full flex items-center justify-center font-bold">
                  1
                </div>
                <div>
                  <h4 className="font-semibold text-gray-900">
                    Generate KML Files
                  </h4>
                  <p className="text-sm text-gray-600">
                    Use the KML Generator to draw shapes and export them as KML
                    files
                  </p>
                </div>
              </div>
              <div className="flex items-start space-x-3">
                <div className="flex-shrink-0 h-8 w-8 bg-blue-600 text-white rounded-full flex items-center justify-center font-bold">
                  2
                </div>
                <div>
                  <h4 className="font-semibold text-gray-900">View on Map</h4>
                  <p className="text-sm text-gray-600">
                    Load your KML files in the viewer to visualize them on
                    OpenStreetMap
                  </p>
                </div>
              </div>
              <div className="flex items-start space-x-3">
                <div className="flex-shrink-0 h-8 w-8 bg-purple-600 text-white rounded-full flex items-center justify-center font-bold">
                  3
                </div>
                <div>
                  <h4 className="font-semibold text-gray-900">AI Navigation</h4>
                  <p className="text-sm text-gray-600">
                    Use the AI Agent to automatically navigate through your
                    waypoints
                  </p>
                </div>
              </div>
            </div>
          </CardContent>
        </Card>

        <div className="mt-8 grid gap-4 md:grid-cols-2">
          <Card>
            <CardHeader>
              <CardTitle className="text-lg">Recent Activity</CardTitle>
            </CardHeader>
            <CardContent>
              <p className="text-sm text-gray-600">
                Your activity will appear here once you start using the tools
              </p>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle className="text-lg">System Status</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-2 text-sm">
                <div className="flex justify-between">
                  <span className="text-gray-600">Map Service:</span>
                  <span className="text-green-600 font-medium">✓ Online</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-600">Database:</span>
                  <span className="text-green-600 font-medium">
                    ✓ Connected
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-600">API Status:</span>
                  <span className="text-green-600 font-medium">✓ Ready</span>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}
