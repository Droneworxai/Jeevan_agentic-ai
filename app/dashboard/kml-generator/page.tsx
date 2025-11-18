'use client';

import { useState, useEffect, useRef } from 'react';
import MapboxDrawMapComponent from '@/components/map/mapbox-draw-map-component';
import {
  Card,
  CardContent,
} from '@/components/ui/card';
import { Autocomplete } from '@/components/search/Search';
import L from 'leaflet';

export default function KMLGeneratorPage() {
  const [mounted, setMounted] = useState(false);
  const mapRef = useRef<L.Map | null>(null);

  useEffect(() => {
    setMounted(true);
  }, []);

  const getNominatimSuggestions = ({ query }: { query: string }) => {
    if (!query) {
      return Promise.resolve([]);
    }

    const url = new URL('https://nominatim.openstreetmap.org/search');
    url.searchParams.set('q', query);
    url.searchParams.set('format', 'json');
    url.searchParams.set('limit', '5');

    return fetch(url.toString())
      .then((response) => response.json())
      .then((data) => {
        return data.map((item: any) => ({
          name: item.display_name,
          lat: item.lat,
          lon: item.lon,
          boundingbox: item.boundingbox,
        }));
      });
  };

  const handleSearchSelect = (item: any) => {
    if (mapRef.current && item.boundingbox) {
        const [minLat, maxLat, minLon, maxLon] = item.boundingbox.map(parseFloat);
        const bounds = L.latLngBounds([minLat, minLon], [maxLat, maxLon]);
        mapRef.current.fitBounds(bounds);
    } else if (mapRef.current) {
        mapRef.current.panTo([item.lat, item.lon]);
        mapRef.current.setZoom(15);
    }
  };

  if (!mounted) {
    return (
      <div className='h-full flex items-center justify-center'>
        <div className='text-center'>
          <div className='animate-spin rounded-full h-12 w-12 border-b-2 border-green-600 mx-auto mb-4'></div>
          <p className='text-gray-600'>Loading map...</p>
        </div>
      </div>
    );
  }

  return (
    <div className='flex flex-col h-full'>
      {/* Spacer for the fixed header */}
      <div className='h-16 flex-shrink-0' /> 
      
      <div className='p-6 flex-grow flex flex-col'>
        {/* Page Description */}
        <div className='mb-4'>
            <h2 className='text-2xl font-bold text-gray-900'>KML Generator</h2>
            <p className='text-gray-600 mt-1 text-sm'>
                Create and export KML files by drawing on the map. Use the search bar to find a location, then use the drawing tools on the map to create features. Once you&apos;re done, you can download the KML file from the panel on the right.
            </p>
        </div>

        {/* Map Card */}
        <Card className='flex-grow shadow-lg'>
            <CardContent className='p-0 h-full'>
              <MapboxDrawMapComponent mapRef={mapRef}/>
            </CardContent>
        </Card>

        {/* Search Box */}
        <div className="relative z-10 mt-4">
            <Autocomplete
              placeholder="Search for a location to begin"
              className="[&_.aa-Input]:text-gray-900"
              getSources={({ query }) => [
                {
                  sourceId: 'places',
                  getItems() {
                    return getNominatimSuggestions({ query });
                  },
                  templates: {
                    item({ item, ...props }) {
                      return (
                        <div 
                            {...props}
                            className='p-2 border-b border-gray-200 bg-white cursor-pointer hover:bg-gray-100 text-black'
                        >
                        <span>{item.name}</span>
                        </div>
                      );
                    },
                    empty({ query, ...props }: { query: string; [key: string]: any }) {
                        return (
                            <div {...props} className='p-4 bg-white text-center text-gray-500'>
                                No results for &quot;{query}&quot;.
                            </div>
                        )
                    }
                  },
                  onSelect: ({ item }) => {
                    handleSearchSelect(item);
                  },
                },
              ]}
            />
        </div>

        {/* Search Features Description */}
        <div className="mt-4">
            <h3 className="text-lg font-semibold text-gray-900">Search Features:</h3>
            <ul className="list-disc list-inside text-gray-600 text-sm mt-2 space-y-1">
                <li><strong>Location Search:</strong> Quickly find any location using OpenStreetMap&apos;s extensive database.</li>
                <li><strong>Auto-Suggestions:</strong> As you type, you get instant location suggestions to speed up your search.</li>
                <li><strong>Powered by Algolia:</strong> A smooth and fast search experience is provided by Algolia&apos;s Autocomplete library.</li>
                <li><strong>Seamless Map Integration:</strong> Selecting a location will automatically pan and zoom the map to that area.</li>
            </ul>
        </div>
      </div>
    </div>
  );
}