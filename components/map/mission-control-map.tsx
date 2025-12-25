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
  onPathLinesChange?: (lines: [number, number][][]) => void;
  robotPosition: [number, number] | null;
  aiWaypoints?: [number, number][];
  currentWaypointIndex?: number;
}

export default function MissionControlMap({ 
  onBoundaryChange,
  onPathLinesChange,
  robotPosition,
  aiWaypoints = [],
  currentWaypointIndex = 0
}: MissionControlMapProps) {
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const drawnItemsRef = useRef<L.FeatureGroup | null>(null);
  const pathLinesRef = useRef<L.FeatureGroup | null>(null);
  const robotMarkerRef = useRef<L.Marker | null>(null);
  const pathLineRef = useRef<L.Polyline | null>(null);
  const pathCoordsRef = useRef<L.LatLng[]>([]);
  const waypointMarkersRef = useRef<L.Marker[]>([]);
  const waypointPathRef = useRef<L.Polyline | null>(null);
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  useEffect(() => {
    if (!mounted || !mapContainerRef.current || mapRef.current) return;

    const map = L.map(mapContainerRef.current).setView([52.175, -1.755], 16);
    mapRef.current = map;

    // Enhanced OpenStreetMap tile layer with better styling
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      maxZoom: 19,
      minZoom: 10,
      crossOrigin: true,
    }).addTo(map);

    // Add scale control
    L.control.scale({ imperial: false, metric: true }).addTo(map);

    const drawnItems = new L.FeatureGroup();
    map.addLayer(drawnItems);
    drawnItemsRef.current = drawnItems;

    // Separate layer for path lines inside the boundary
    const pathLinesLayer = new L.FeatureGroup();
    map.addLayer(pathLinesLayer);
    pathLinesRef.current = pathLinesLayer;

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

    // Add custom control for drawing path lines
    const pathDrawControl = new L.Control.Draw({
      position: 'topright',
      edit: {
        featureGroup: pathLinesLayer,
        remove: true,
      },
      draw: {
        polyline: {
          shapeOptions: { 
            color: "#f59e0b", 
            weight: 4,
            opacity: 0.8,
            dashArray: '10, 10'
          },
          showLength: true,
          metric: true,
        },
        polygon: false,
        rectangle: false,
        circle: false,
        circlemarker: false,
        marker: false,
      },
    });
    map.addControl(pathDrawControl);

    const updateGeoJSON = () => {
      if (drawnItemsRef.current) {
        const data = drawnItemsRef.current.toGeoJSON();
        onBoundaryChange(data);
      }
    };

    const updatePathLines = () => {
      if (pathLinesRef.current && onPathLinesChange) {
        const lines: [number, number][][] = [];
        pathLinesRef.current.eachLayer((layer: any) => {
          if (layer instanceof L.Polyline) {
            const coords = layer.getLatLngs().map((latlng: any) => 
              [latlng.lat, latlng.lng] as [number, number]
            );
            lines.push(coords);
          }
        });
        onPathLinesChange(lines);
      }
    };

    map.on(L.Draw.Event.CREATED, (e: any) => {
      const layer = e.layer;
      const layerType = e.layerType;
      
      if (layerType === 'polyline') {
        // Add to path lines layer
        pathLinesLayer.addLayer(layer);
        
        // Add popup showing this is a robot path
        layer.bindPopup(`
          <div style="text-align: center;">
            <strong>🤖 Robot Path Line</strong><br/>
            <small>Robot will follow this path slowly</small><br/>
            <small>Length: ${(layer.getDistance() || 0).toFixed(2)}m</small>
          </div>
        `);
        
        updatePathLines();
      } else {
        // Add to boundary layer
        drawnItems.addLayer(layer);
        updateGeoJSON();
      }
    });

    map.on(L.Draw.Event.EDITED, (e: any) => {
      updateGeoJSON();
      updatePathLines();
    });
    
    map.on(L.Draw.Event.DELETED, (e: any) => {
      updateGeoJSON();
      updatePathLines();
    });

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, [mounted, onBoundaryChange]);

  // Update robot position on map
  useEffect(() => {
    if (!mapRef.current || !robotPosition) return;

    console.log("Robot position update:", robotPosition); // Debug log

    if (!robotMarkerRef.current) {
      const botIcon = L.divIcon({
        className: 'custom-div-icon',
        html: `<div style="
          background: linear-gradient(135deg, #10b981 0%, #059669 100%);
          width: 32px;
          height: 32px;
          border-radius: 8px;
          border: 3px solid white;
          box-shadow: 0 4px 12px rgba(0,0,0,0.3);
          display: flex;
          align-items: center;
          justify-content: center;
          transform: rotate(0deg);
          z-index: 1000;
        ">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="white" stroke-width="2.5">
            <path d="M5 11a2 2 0 0 0-2 2v4a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-4a2 2 0 0 0-2-2H5Z"/>
            <path d="M6 8V6a2 2 0 0 1 2-2h8a2 2 0 0 1 2 2v2"/>
            <circle cx="7" cy="17" r="1"/>
            <circle cx="17" cy="17" r="1"/>
            <path d="M9 11h6"/>
          </svg>
        </div>`,
        iconSize: [32, 32],
        iconAnchor: [16, 16]
      });
      robotMarkerRef.current = L.marker(robotPosition, { 
        icon: botIcon,
        zIndexOffset: 1000 
      }).addTo(mapRef.current);
      
      // Pan to robot position
      mapRef.current.panTo(robotPosition);
    } else {
      robotMarkerRef.current.setLatLng(robotPosition);
    }

    // Update path visualization
    const newLatLng = L.latLng(robotPosition[0], robotPosition[1]);
    
    // Only add if position changed significantly (e.g., > 0.5m) to save performance
    const lastCoord = pathCoordsRef.current[pathCoordsRef.current.length - 1];
    if (!lastCoord || lastCoord.distanceTo(newLatLng) > 0.5) {
      pathCoordsRef.current.push(newLatLng);
      
      if (!pathLineRef.current) {
        pathLineRef.current = L.polyline(pathCoordsRef.current, {
          color: '#3b82f6',
          weight: 2,
          opacity: 0.6,
          dashArray: '5, 10'
        }).addTo(mapRef.current);
      } else {
        pathLineRef.current.setLatLngs(pathCoordsRef.current);
      }
    }
  }, [robotPosition]);

  // Visualize AI navigation waypoints
  useEffect(() => {
    if (!mapRef.current) return;

    // Clear existing waypoint markers
    waypointMarkersRef.current.forEach(marker => {
      if (mapRef.current) {
        mapRef.current.removeLayer(marker);
      }
    });
    waypointMarkersRef.current = [];

    // Clear existing waypoint path
    if (waypointPathRef.current && mapRef.current) {
      mapRef.current.removeLayer(waypointPathRef.current);
      waypointPathRef.current = null;
    }

    if (aiWaypoints.length === 0) return;

    // Add waypoint markers
    aiWaypoints.forEach((waypoint, index) => {
      const isCompleted = index < currentWaypointIndex;
      const isCurrent = index === currentWaypointIndex;
      
      const waypointIcon = L.divIcon({
        className: 'custom-waypoint-icon',
        html: `<div style="
          background: ${isCompleted ? '#10b981' : isCurrent ? '#3b82f6' : '#9ca3af'};
          width: 24px;
          height: 24px;
          border-radius: 50%;
          border: 3px solid white;
          box-shadow: 0 2px 8px rgba(0,0,0,0.3);
          display: flex;
          align-items: center;
          justify-content: center;
          font-size: 10px;
          font-weight: bold;
          color: white;
        ">${index + 1}</div>`,
        iconSize: [24, 24],
        iconAnchor: [12, 12]
      });

      const marker = L.marker([waypoint[0], waypoint[1]], { 
        icon: waypointIcon 
      }).addTo(mapRef.current!);
      
      marker.bindPopup(`
        <strong>Waypoint ${index + 1}</strong><br/>
        Status: ${isCompleted ? '✓ Completed' : isCurrent ? '→ Current' : '○ Pending'}<br/>
        Lat: ${waypoint[0].toFixed(6)}<br/>
        Lng: ${waypoint[1].toFixed(6)}
      `);
      
      waypointMarkersRef.current.push(marker);
    });

    // Draw path connecting waypoints
    const waypointLatLngs = aiWaypoints.map(wp => L.latLng(wp[0], wp[1]));
    waypointPathRef.current = L.polyline(waypointLatLngs, {
      color: '#8b5cf6',
      weight: 3,
      opacity: 0.7,
      dashArray: '10, 5',
      lineCap: 'round'
    }).addTo(mapRef.current);

    // Fit map bounds to show all waypoints
    if (waypointLatLngs.length > 0) {
      const bounds = L.latLngBounds(waypointLatLngs);
      mapRef.current.fitBounds(bounds, { padding: [50, 50] });
    }
  }, [aiWaypoints, currentWaypointIndex]);

  return <div ref={mapContainerRef} className="w-full h-full min-h-[500px] rounded-lg shadow-inner" />;
}
