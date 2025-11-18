"use client";

import { useEffect, useRef, useState } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/labels";
import { Upload, Car, Play, Pause, Settings, Trash2 } from "lucide-react";
import { parseKMLToGeoJSON } from "@/lib/kml-parser";

export default function AIAgentMapComponent() {
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const kmlLayerRef = useRef<L.GeoJSON | null>(null);
  const carMarkerRef = useRef<L.Marker | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const [mounted, setMounted] = useState(false);
  const [kmlLoaded, setKmlLoaded] = useState(false);
  const [kmlData, setKmlData] = useState<any>(null);
  const [carAdded, setCarAdded] = useState(false);
  const [geminiApiKey, setGeminiApiKey] = useState("");
  const [apiKeySet, setApiKeySet] = useState(false);
  const [isNavigating, setIsNavigating] = useState(false);
  const [navigationPath, setNavigationPath] = useState<L.LatLng[]>([]);
  const [currentPathIndex, setCurrentPathIndex] = useState(0);

  useEffect(() => {
    setMounted(true);
    const savedKey = localStorage.getItem("geminiApiKey");
    if (savedKey) {
      setGeminiApiKey(savedKey);
      setApiKeySet(true);
    }
  }, []);

  useEffect(() => {
    if (!mounted || !mapContainerRef.current || mapRef.current) return;

    // Layer 1: Initialize map with OpenStreetMap
    const map = L.map(mapContainerRef.current).setView([27.7172, 85.324], 13);
    mapRef.current = map;

    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: "© OpenStreetMap contributors",
      maxZoom: 19,
    }).addTo(map);

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, [mounted]);

  // Navigation animation
  useEffect(() => {
    if (!isNavigating || !carMarkerRef.current || navigationPath.length === 0)
      return;

    const interval = setInterval(() => {
      setCurrentPathIndex((prev) => {
        if (prev >= navigationPath.length - 1) {
          setIsNavigating(false);
          return prev;
        }

        const nextIndex = prev + 1;
        const nextPosition = navigationPath[nextIndex];

        if (carMarkerRef.current) {
          carMarkerRef.current.setLatLng(nextPosition);
          mapRef.current?.panTo(nextPosition);
        }

        return nextIndex;
      });
    }, 1000);

    return () => clearInterval(interval);
  }, [isNavigating, navigationPath]);

  const handleSaveApiKey = () => {
    if (geminiApiKey.trim()) {
      localStorage.setItem("geminiApiKey", geminiApiKey);
      setApiKeySet(true);
      alert("Gemini API Key saved successfully!");
    }
  };

  const handleFileSelect = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = async (e) => {
      try {
        const kmlText = e.target?.result as string;
        const geoJSON = parseKMLToGeoJSON(kmlText);

        if (mapRef.current) {
          if (kmlLayerRef.current) {
            mapRef.current.removeLayer(kmlLayerRef.current);
          }

          // Layer 2: KML data layer
          const kmlLayer = L.geoJSON(geoJSON, {
            style: {
              color: "#ef4444",
              weight: 3,
              opacity: 0.7,
              fillOpacity: 0.3,
            },
            pointToLayer: (feature, latlng) => {
              return L.circleMarker(latlng, {
                radius: 8,
                fillColor: "#ef4444",
                color: "#991b1b",
                weight: 2,
                opacity: 1,
                fillOpacity: 0.8,
              });
            },
            onEachFeature: (feature, layer) => {
              if (feature.properties && feature.properties.name) {
                layer.bindPopup(`<strong>${feature.properties.name}</strong>`);
              }
            },
          });

          kmlLayer.addTo(mapRef.current);
          kmlLayerRef.current = kmlLayer;

          const bounds = kmlLayer.getBounds();
          if (bounds.isValid()) {
            mapRef.current.fitBounds(bounds, { padding: [50, 50] });
          }

          setKmlLoaded(true);
          setKmlData(geoJSON);
        }
      } catch (error) {
        console.error("Error parsing KML:", error);
        alert("Error loading KML file.");
      }
    };
    reader.readAsText(file);
  };

  const handleAddCar = () => {
    if (!mapRef.current) return;

    const center = mapRef.current.getCenter();

    // Layer 3: Car icon marker
    const carIcon = L.divIcon({
      html: `<div style="font-size: 30px;">🚗</div>`,
      className: "car-icon",
      iconSize: [30, 30],
      iconAnchor: [15, 15],
    });

    if (carMarkerRef.current) {
      mapRef.current.removeLayer(carMarkerRef.current);
    }

    const carMarker = L.marker(center, {
      icon: carIcon,
      draggable: true,
    }).addTo(mapRef.current);

    carMarker.bindPopup("AI Navigation Vehicle");
    carMarkerRef.current = carMarker;
    setCarAdded(true);
  };

  const handleStartNavigation = async () => {
    if (!apiKeySet) {
      alert("Please set your Gemini API key first!");
      return;
    }

    if (!kmlLoaded) {
      alert("Please load a KML file first!");
      return;
    }

    if (!carAdded || !carMarkerRef.current) {
      alert("Please add a car to the map first!");
      return;
    }

    setIsNavigating(true);
    setCurrentPathIndex(0);

    // Extract waypoints from KML data
    const waypoints: L.LatLng[] = [];

    if (kmlData && kmlData.features) {
      kmlData.features.forEach((feature: any) => {
        if (feature.geometry.type === "Point") {
          const [lng, lat] = feature.geometry.coordinates;
          waypoints.push(L.latLng(lat, lng));
        } else if (feature.geometry.type === "LineString") {
          feature.geometry.coordinates.forEach((coord: number[]) => {
            waypoints.push(L.latLng(coord[1], coord[0]));
          });
        } else if (feature.geometry.type === "Polygon") {
          feature.geometry.coordinates[0].forEach((coord: number[]) => {
            waypoints.push(L.latLng(coord[1], coord[0]));
          });
        }
      });
    }

    if (waypoints.length === 0) {
      alert("No waypoints found in KML file!");
      setIsNavigating(false);
      return;
    }

    // Simple pathfinding - connect all waypoints
    const startPosition = carMarkerRef.current.getLatLng();
    const fullPath = [startPosition, ...waypoints];

    setNavigationPath(fullPath);
  };

  const handlePauseNavigation = () => {
    setIsNavigating(false);
  };

  const handleClearAll = () => {
    if (kmlLayerRef.current && mapRef.current) {
      mapRef.current.removeLayer(kmlLayerRef.current);
      kmlLayerRef.current = null;
    }
    if (carMarkerRef.current && mapRef.current) {
      mapRef.current.removeLayer(carMarkerRef.current);
      carMarkerRef.current = null;
    }
    setKmlLoaded(false);
    setKmlData(null);
    setCarAdded(false);
    setIsNavigating(false);
    setNavigationPath([]);
    setCurrentPathIndex(0);
    if (fileInputRef.current) {
      fileInputRef.current.value = "";
    }
  };

  return (
    <div className="flex h-full">
      <div className="flex-1 relative">
        <div ref={mapContainerRef} className="absolute inset-0" />

        <div className="absolute top-4 left-4 z-[1000] space-x-2">
          <input
            ref={fileInputRef}
            type="file"
            accept=".kml"
            onChange={handleFileSelect}
            className="hidden"
          />
          <Button
            onClick={() => fileInputRef.current?.click()}
            className="bg-purple-600 hover:bg-purple-700 shadow-lg"
          >
            <Upload className="h-4 w-4 mr-2" />
            Load KML
          </Button>

          <Button
            onClick={handleAddCar}
            className="bg-blue-600 hover:bg-blue-700 shadow-lg"
            disabled={!kmlLoaded}
          >
            <Car className="h-4 w-4 mr-2" />
            Add Car
          </Button>

          {!isNavigating ? (
            <Button
              onClick={handleStartNavigation}
              className="bg-green-600 hover:bg-green-700 shadow-lg"
              disabled={!carAdded || !kmlLoaded || !apiKeySet}
            >
              <Play className="h-4 w-4 mr-2" />
              Start Navigation
            </Button>
          ) : (
            <Button
              onClick={handlePauseNavigation}
              className="bg-orange-600 hover:bg-orange-700 shadow-lg"
            >
              <Pause className="h-4 w-4 mr-2" />
              Pause
            </Button>
          )}

          <Button
            onClick={handleClearAll}
            variant="destructive"
            className="shadow-lg"
          >
            <Trash2 className="h-4 w-4 mr-2" />
            Clear All
          </Button>
        </div>
      </div>

      <div className="w-96 bg-white border-l border-gray-200 p-4 overflow-auto">
        <div className="space-y-4">
          <div>
            <h3 className="text-lg font-semibold mb-2">AI Agent Control</h3>
            <p className="text-sm text-gray-600">
              Configure AI navigation with Gemini API
            </p>
          </div>

          <div className="border-t pt-4 space-y-3">
            <div>
              <Label htmlFor="gemini-key">Gemini API Key</Label>
              <div className="flex space-x-2 mt-1">
                <Input
                  id="gemini-key"
                  type="password"
                  placeholder="Enter your Gemini API key"
                  value={geminiApiKey}
                  onChange={(e) => setGeminiApiKey(e.target.value)}
                  disabled={apiKeySet}
                />
                {!apiKeySet ? (
                  <Button onClick={handleSaveApiKey} size="sm">
                    <Settings className="h-4 w-4" />
                  </Button>
                ) : (
                  <Button
                    onClick={() => {
                      setApiKeySet(false);
                      setGeminiApiKey("");
                      localStorage.removeItem("geminiApiKey");
                    }}
                    variant="outline"
                    size="sm"
                  >
                    Change
                  </Button>
                )}
              </div>
              {apiKeySet && (
                <p className="text-xs text-green-600 mt-1">
                  ✓ API Key configured
                </p>
              )}
            </div>
          </div>

          <div className="border-t pt-4">
            <h4 className="font-medium mb-2">Status</h4>
            <div className="space-y-2 text-sm">
              <div className="flex justify-between">
                <span className="text-gray-600">KML Loaded:</span>
                <span
                  className={kmlLoaded ? "text-green-600" : "text-gray-400"}
                >
                  {kmlLoaded ? "✓ Yes" : "✗ No"}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-600">Car Added:</span>
                <span className={carAdded ? "text-green-600" : "text-gray-400"}>
                  {carAdded ? "✓ Yes" : "✗ No"}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-600">API Key Set:</span>
                <span
                  className={apiKeySet ? "text-green-600" : "text-gray-400"}
                >
                  {apiKeySet ? "✓ Yes" : "✗ No"}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-600">Navigation:</span>
                <span
                  className={isNavigating ? "text-blue-600" : "text-gray-400"}
                >
                  {isNavigating ? "▶ Active" : "⏸ Idle"}
                </span>
              </div>
            </div>
          </div>

          {navigationPath.length > 0 && (
            <div className="border-t pt-4">
              <h4 className="font-medium mb-2">Navigation Progress</h4>
              <div className="bg-gray-100 rounded-full h-2 mb-2">
                <div
                  className="bg-blue-600 h-2 rounded-full transition-all duration-300"
                  style={{
                    width: `${
                      (currentPathIndex / (navigationPath.length - 1)) * 100
                    }%`,
                  }}
                />
              </div>
              <p className="text-xs text-gray-600">
                Waypoint {currentPathIndex + 1} of {navigationPath.length}
              </p>
            </div>
          )}

          <div className="border-t pt-4">
            <h4 className="font-medium mb-2">Map Layers</h4>
            <ul className="text-sm space-y-1 text-gray-700">
              <li>• Layer 1: OpenStreetMap (Background)</li>
              <li>• Layer 2: KML Data (Red Markers)</li>
              <li>• Layer 3: AI Vehicle (Movable Car)</li>
            </ul>
          </div>

          <div className="border-t pt-4">
            <h4 className="font-medium mb-2">Instructions</h4>
            <ol className="text-xs space-y-1 text-gray-600 list-decimal list-inside">
              <li>Set your Gemini API key</li>
              <li>Load a KML file with waypoints</li>
              <li>Add a car to the map</li>
              <li>Click "Start Navigation" to begin</li>
              <li>The car will navigate through waypoints</li>
            </ol>
          </div>
        </div>
      </div>
    </div>
  );
}
