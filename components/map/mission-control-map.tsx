"use client";

import { useEffect, useRef, useState } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import "leaflet-draw/dist/leaflet.draw.css";
import "leaflet-draw";
import ROSLIB from "roslib";
import RosClient from "@/lib/ros/ros-client";

interface MissionControlMapProps {
  onBoundaryChange: (geoJSON: any) => void;
  robotPosition: [number, number] | null;
}

export default function MissionControlMap({ onBoundaryChange, robotPosition }: MissionControlMapProps) {
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const drawnItemsRef = useRef<L.FeatureGroup | null>(null);
  const robotMarkerRef = useRef<L.Marker | null>(null);
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  useEffect(() => {
    if (!mounted || !mapContainerRef.current || mapRef.current) return;

    const map = L.map(mapContainerRef.current).setView([27.7172, 85.324], 13);
    mapRef.current = map;

    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      maxZoom: 19,
    }).addTo(map);

    const drawnItems = new L.FeatureGroup();
    map.addLayer(drawnItems);
    drawnItemsRef.current = drawnItems;

    const drawControl = new L.Control.Draw({
      edit: {
        featureGroup: drawnItems,
        remove: true,
      },
      draw: {
        polygon: {
          allowIntersection: false,
          shapeOptions: { color: "#16a34a", weight: 3 },
        },
        rectangle: {
          shapeOptions: { color: "#16a34a", weight: 3 },
        },
        circle: false,
        circlemarker: false,
        marker: false,
        polyline: false,
      },
    });
    map.addControl(drawControl);

    const updateGeoJSON = () => {
      if (drawnItemsRef.current) {
        const data = drawnItemsRef.current.toGeoJSON();
        onBoundaryChange(data);
      }
    };

    map.on(L.Draw.Event.CREATED, (e: any) => {
      drawnItems.addLayer(e.layer);
      updateGeoJSON();
    });

    map.on(L.Draw.Event.EDITED, updateGeoJSON);
    map.on(L.Draw.Event.DELETED, updateGeoJSON);

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, [mounted, onBoundaryChange]);

  // Update robot position on map
  useEffect(() => {
    if (!mapRef.current || !robotPosition) return;

    if (!robotMarkerRef.current) {
      const botIcon = L.divIcon({
        className: 'custom-div-icon',
        html: "<div style='background-color:#ef4444;width:12px;height:12px;border-radius:50%;border:2px solid white;'></div>",
        iconSize: [12, 12],
        iconAnchor: [6, 6]
      });
      robotMarkerRef.current = L.marker(robotPosition, { icon: botIcon }).addTo(mapRef.current);
    } else {
      robotMarkerRef.current.setLatLng(robotPosition);
    }
  }, [robotPosition]);

  return <div ref={mapContainerRef} className="w-full h-full min-h-[500px] rounded-lg shadow-inner" />;
}
