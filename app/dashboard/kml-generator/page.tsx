'use client';

import { useState, useEffect, useRef } from 'react';
import dynamic from 'next/dynamic';
import {
  Card,
  CardContent,
} from '@/components/ui/card';
import { Autocomplete } from '@/components/search/Search';
import type L from 'leaflet';
import ErrorBoundary from '@/components/ui/error-boundary';

const MapboxDrawMapComponent = dynamic(
  () => import('@/components/map/mapbox-draw-map-component'),
  { ssr: false }
);

export default function KMLGeneratorPage() {
  const [mounted, setMounted] = useState(false);
  const mapRef = useRef<L.Map | null>(null);

  useEffect(() => {
    setMounted(true);
  }, []);

  const getSearchSuggestions = ({ query }: { query: string }) => {
    if (!query) {
      return Promise.resolve([]);
    }

    const url = new URL('/api/search', window.location.origin);
    url.searchParams.set('q', query);

    return fetch(url.toString())
      .then((response) => {
        if (!response.ok) {
          throw new Error('Search failed');
        }
        return response.json();
      })
      .then((data) => {
        if (data.error) {
          console.error('Search API error:', data.error);
          return [];
        }
        return data.map((item: any) => ({
          name: item.display_name,
          lat: item.lat,
          lon: item.lon,
          boundingbox: item.boundingbox,
        }));
      })
      .catch((err) => {
        console.error('Search fetch error:', err);
        return [];
      });
  };

  const handleSearchSelect = (item: any) => {
    if (mapRef.current && item.boundingbox) {
        const [minLat, maxLat, minLon, maxLon] = item.boundingbox.map(parseFloat);
        // Leaflet fitBounds expects [[lat, lon], [lat, lon]]
        mapRef.current.fitBounds([
          [minLat, minLon],
          [maxLat, maxLon]
        ]);
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
              <ErrorBoundary>
                <MapboxDrawMapComponent mapRef={mapRef}/>
              </ErrorBoundary>
            </CardContent>
        </Card>

        {/* Search Box */}
        <div className="relative z-10 mt-4">
            <Autocomplete
              placeholder="Search for a location (e.g. United Kingdom)"
              className="[&_.aa-Input]:text-gray-900"
              openOnFocus={true}
              autoFocus={true}
              getSources={({ query }) => [
                {
                  sourceId: 'places',
                  getItems() {
                    return getSearchSuggestions({ query });
                  },
                  templates: {
                    item({ item, components }) {
                      return (
                        <div className="flex items-center p-2 hover:bg-gray-100 cursor-pointer text-black">
                          <div className="flex flex-col">
                            <span className="font-medium">{item.name}</span>
                          </div>
                        </div>
                      );
                    },
                    empty({ query }) {
                        return (
                            <div className='p-4 bg-white text-center text-gray-500'>
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