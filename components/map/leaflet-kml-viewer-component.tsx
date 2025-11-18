"use client";

import { useEffect, useRef, useState } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { Button } from "@/components/ui/button";
import { Upload, Trash2 } from "lucide-react";
import { parseKMLToGeoJSON } from "@/lib/kml-parser";

export default function LeafletKMLViewerComponent() {
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const kmlLayerRef = useRef<L.GeoJSON | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [mounted, setMounted] = useState(false);
  const [kmlLoaded, setKmlLoaded] = useState(false);
  const [kmlData, setKmlData] = useState<any>(null);

  useEffect(() => {
    setMounted(true);
  }, []);

  useEffect(() => {
    if (!mounted || !mapContainerRef.current || mapRef.current) return;

    // Initialize map - Layer 1: OpenStreetMap background
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

  const handleFileSelect = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = async (e) => {
      try {
        const kmlText = e.target?.result as string;
        const geoJSON = parseKMLToGeoJSON(kmlText);

        if (mapRef.current) {
          // Remove existing KML layer if present
          if (kmlLayerRef.current) {
            mapRef.current.removeLayer(kmlLayerRef.current);
          }

          // Layer 2: Add KML data as GeoJSON layer
          const kmlLayer = L.geoJSON(geoJSON, {
            style: {
              color: "#3b82f6",
              weight: 3,
              opacity: 0.7,
              fillOpacity: 0.3,
            },
            pointToLayer: (feature, latlng) => {
              return L.circleMarker(latlng, {
                radius: 8,
                fillColor: "#3b82f6",
                color: "#1e40af",
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

          // Fit map bounds to KML data
          const bounds = kmlLayer.getBounds();
          if (bounds.isValid()) {
            mapRef.current.fitBounds(bounds, { padding: [50, 50] });
          }

          setKmlLoaded(true);
          setKmlData(geoJSON);
        }
      } catch (error) {
        console.error("Error parsing KML:", error);
        alert("Error loading KML file. Please ensure it is a valid KML file.");
      }
    };
    reader.readAsText(file);
  };

  const handleClearKML = () => {
    if (mapRef.current && kmlLayerRef.current) {
      mapRef.current.removeLayer(kmlLayerRef.current);
      kmlLayerRef.current = null;
      setKmlLoaded(false);
      setKmlData(null);
    }
    if (fileInputRef.current) {
      fileInputRef.current.value = "";
    }
  };

  const handleUploadClick = () => {
    fileInputRef.current?.click();
  };

  return (
    <div className="flex h-full">
      <div className="flex-1 relative">
        <div ref={mapContainerRef} className="absolute inset-0" />
        <div className="absolute top-4 left-4 z-[1000]">
          <input
            ref={fileInputRef}
            type="file"
            accept=".kml"
            onChange={handleFileSelect}
            className="hidden"
          />
          <Button
            onClick={handleUploadClick}
            className="bg-blue-600 hover:bg-blue-700 shadow-lg"
          >
            <Upload className="h-4 w-4 mr-2" />
            Open KML File
          </Button>
        </div>
        {kmlLoaded && (
          <div className="absolute top-4 left-44 z-[1000]">
            <Button
              onClick={handleClearKML}
              variant="destructive"
              className="shadow-lg"
            >
              <Trash2 className="h-4 w-4 mr-2" />
              Clear KML
            </Button>
          </div>
        )}
      </div>
      <div className="w-80 bg-white border-l border-gray-200 p-4 overflow-auto">
        <div className="space-y-4">
          <div>
            <h3 className="text-lg font-semibold mb-2">KML Viewer</h3>
            <p className="text-sm text-gray-600">
              Upload a KML file to visualize it on the map.
            </p>
          </div>

          {kmlLoaded ? (
            <>
              <div className="p-3 bg-green-50 border border-green-200 rounded-md">
                <p className="text-sm text-green-800 font-medium">
                  ✓ KML file loaded successfully
                </p>
              </div>

              <div className="border-t pt-4">
                <h4 className="font-medium mb-2">Loaded Features</h4>
                <div className="bg-gray-50 p-3 rounded-md">
                  <p className="text-sm text-gray-700">
                    Total features: {kmlData?.features?.length || 0}
                  </p>
                </div>
              </div>

              <div className="border-t pt-4">
                <h4 className="font-medium mb-2">GeoJSON Data</h4>
                <div className="bg-gray-50 p-3 rounded-md max-h-96 overflow-auto">
                  <pre className="text-xs text-gray-700 whitespace-pre-wrap break-words">
                    {JSON.stringify(kmlData, null, 2)}
                  </pre>
                </div>
              </div>
            </>
          ) : (
            <div className="p-4 bg-gray-50 border-2 border-dashed border-gray-300 rounded-lg text-center">
              <Upload className="h-12 w-12 text-gray-400 mx-auto mb-2" />
              <p className="text-sm text-gray-600">No KML file loaded</p>
              <p className="text-xs text-gray-500 mt-1">
                Click the button above to upload
              </p>
            </div>
          )}

          <div className="border-t pt-4">
            <h4 className="font-medium mb-2">Map Layers</h4>
            <ul className="text-sm space-y-1 text-gray-700">
              <li>• Layer 1: OpenStreetMap (Background)</li>
              <li>• Layer 2: KML Data (Loaded Points)</li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
}
