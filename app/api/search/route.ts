import { NextRequest, NextResponse } from 'next/server';

export async function GET(request: NextRequest) {
  const { searchParams } = new URL(request.url);
  const query = searchParams.get('q');

  if (!query) {
    return NextResponse.json([]);
  }

  const url = new URL('https://photon.komoot.io/api/');
  url.searchParams.set('q', query);
  url.searchParams.set('limit', '5');

  try {
    const response = await fetch(url.toString(), {
      headers: {
        'User-Agent': 'Ecoweeder/1.0 (https://github.com/jeevan-koiri/droneworx)',
        'Accept-Language': 'en',
      },
      next: { revalidate: 3600 } // Cache for 1 hour
    });

    if (!response.ok) {
      throw new Error(`Search API responded with status: ${response.status}`);
    }

    const data = await response.json();
    // Map Photon format to a simpler format
    const results = data.features.map((feature: any) => ({
      display_name: [
        feature.properties.name,
        feature.properties.city,
        feature.properties.state,
        feature.properties.country
      ].filter(Boolean).join(', '),
      lat: feature.geometry.coordinates[1],
      lon: feature.geometry.coordinates[0],
      boundingbox: feature.properties.extent ? [
        feature.properties.extent[1],
        feature.properties.extent[3],
        feature.properties.extent[0],
        feature.properties.extent[2]
      ] : null
    }));
    return NextResponse.json(results);
  } catch (error) {
    console.error('Search proxy error:', error);
    return NextResponse.json({ error: 'Failed to fetch search results' }, { status: 500 });
  }
}
