"use client";

import Link from "next/link";
import { useRouter } from "next/navigation";
import { Download, MapPin, Bot, Info, LogOut, Settings, Activity } from "lucide-react";
import { Button } from "@/components/ui/button";
import { cn } from "@/lib/utils";

interface DashboardSidebarProps {
  isOpen: boolean;
  currentPath: string;
}

const navigationItems = [
  {
    title: "KML Downloader",
    href: "/dashboard/kml-generator",
    icon: Download,
    description: "Generate and download KML files",
  },
  {
    title: "KML Opener",
    href: "/dashboard/kml-viewer",
    icon: MapPin,
    description: "View KML files on map",
  },
  {
    title: "AI Agent",
    href: "/dashboard/ai-agent",
    icon: Bot,
    description: "AI-powered navigation",
  },
  {
    title: "EcoWeeder Simulation",
    href: "/dashboard/simulation",
    icon: Activity,
    description: "Robot simulation and control",
  },
  {
    title: "About Us",
    href: "/dashboard/about",
    icon: Info,
    description: "Learn more about Ecoweeder",
  },
];

export default function DashboardSidebar({
  isOpen,
  currentPath,
}: DashboardSidebarProps) {
  const router = useRouter();

  const handleLogout = () => {
    localStorage.removeItem("user");
    localStorage.removeItem("geminiApiKey");
    router.push("/auth/login");
  };

  return (
    <aside
      className={cn(
        "fixed left-0 top-16 h-[calc(100vh-64px)] bg-white border-r border-gray-200 transition-all duration-300 z-40",
        isOpen ? "w-64 translate-x-0" : "w-64 -translate-x-full"
      )}
    >
      <div className="flex flex-col h-full">
        <nav className="flex-1 p-4 space-y-2">
          {navigationItems.map((item) => {
            const Icon = item.icon;
            const isActive = currentPath === item.href;

            return (
              <Link
                key={item.href}
                href={item.href}
                className={cn(
                  "flex items-center space-x-3 px-4 py-3 rounded-lg transition-colors",
                  isActive
                    ? "bg-green-50 text-green-700 font-medium"
                    : "text-gray-700 hover:bg-gray-50"
                )}
              >
                <Icon className="h-5 w-5" />
                <div className="flex-1">
                  <div className="text-sm">{item.title}</div>
                  <div className="text-xs text-gray-500">
                    {item.description}
                  </div>
                </div>
              </Link>
            );
          })}
        </nav>

        <div className="p-4 border-t border-gray-200 space-y-2">
          <Link href="/dashboard/settings">
            <Button variant="outline" className="w-full justify-start">
              <Settings className="h-4 w-4 mr-2" />
              Settings
            </Button>
          </Link>
          <Button
            variant="outline"
            className="w-full justify-start text-red-600 hover:text-red-700 hover:bg-red-50"
            onClick={handleLogout}
          >
            <LogOut className="h-4 w-4 mr-2" />
            Logout
          </Button>
        </div>
      </div>
    </aside>
  );
}
