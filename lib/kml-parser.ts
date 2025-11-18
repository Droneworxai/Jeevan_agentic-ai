export function parseKMLToGeoJSON(kmlText: string): any {
  const parser = new DOMParser();
  const xmlDoc = parser.parseFromString(kmlText, "text/xml");

  const features: any[] = [];

  // Parse Placemarks
  const placemarks = xmlDoc.getElementsByTagName("Placemark");

  for (let i = 0; i < placemarks.length; i++) {
    const placemark = placemarks[i];
    const feature = parsePlacemark(placemark);
    if (feature) {
      features.push(feature);
    }
  }

  return {
    type: "FeatureCollection",
    features: features,
  };
}

function parsePlacemark(placemark: Element): any {
  const nameElement = placemark.getElementsByTagName("name")[0];
  const descriptionElement = placemark.getElementsByTagName("description")[0];

  const properties: any = {
    name: nameElement ? nameElement.textContent : "Unnamed",
    description: descriptionElement ? descriptionElement.textContent : "",
  };

  // Try to parse Point
  const pointElement = placemark.getElementsByTagName("Point")[0];
  if (pointElement) {
    const coordinates = parseCoordinates(pointElement);
    if (coordinates.length > 0) {
      return {
        type: "Feature",
        properties: properties,
        geometry: {
          type: "Point",
          coordinates: coordinates[0],
        },
      };
    }
  }

  // Try to parse LineString
  const lineStringElement = placemark.getElementsByTagName("LineString")[0];
  if (lineStringElement) {
    const coordinates = parseCoordinates(lineStringElement);
    if (coordinates.length > 0) {
      return {
        type: "Feature",
        properties: properties,
        geometry: {
          type: "LineString",
          coordinates: coordinates,
        },
      };
    }
  }

  // Try to parse Polygon
  const polygonElement = placemark.getElementsByTagName("Polygon")[0];
  if (polygonElement) {
    const outerBoundary =
      polygonElement.getElementsByTagName("outerBoundaryIs")[0];
    if (outerBoundary) {
      const linearRing = outerBoundary.getElementsByTagName("LinearRing")[0];
      if (linearRing) {
        const coordinates = parseCoordinates(linearRing);
        if (coordinates.length > 0) {
          return {
            type: "Feature",
            properties: properties,
            geometry: {
              type: "Polygon",
              coordinates: [coordinates],
            },
          };
        }
      }
    }
  }

  return null;
}

function parseCoordinates(element: Element): number[][] {
  const coordinatesElement = element.getElementsByTagName("coordinates")[0];
  if (!coordinatesElement || !coordinatesElement.textContent) {
    return [];
  }

  const coordinatesText = coordinatesElement.textContent.trim();
  const coordinatePairs = coordinatesText.split(/\s+/);

  const coordinates: number[][] = [];

  for (const pair of coordinatePairs) {
    const parts = pair.split(",");
    if (parts.length >= 2) {
      const lon = parseFloat(parts[0]);
      const lat = parseFloat(parts[1]);
      if (!isNaN(lon) && !isNaN(lat)) {
        coordinates.push([lon, lat]);
      }
    }
  }

  return coordinates;
}
