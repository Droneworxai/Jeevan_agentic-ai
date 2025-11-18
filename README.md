# Ecoweeder - KML Generator & AI Navigation Platform

A comprehensive Next.js application for KML file generation, visualization, and AI-powered navigation using Google Gemini API.

## Features

- 🗺️ **KML Generator**: Interactive map drawing with polygon, rectangle, circle, line, and marker tools
- 📍 **KML Viewer**: Load and visualize KML files on OpenStreetMap
- 🤖 **AI Navigation Agent**: AI-powered vehicle navigation through waypoints
- 🔐 **User Authentication**: Secure login and registration system
- 📱 **Responsive Design**: Modern UI built with Tailwind CSS and shadcn/ui
- 🎨 **Three-Layer Map System**:
  - Layer 1: OpenStreetMap background
  - Layer 2: KML data visualization
  - Layer 3: AI-controlled vehicle movement

## Tech Stack

- **Framework**: Next.js 14 with App Router
- **Language**: TypeScript
- **Styling**: Tailwind CSS
- **UI Components**: shadcn/ui with Radix UI
- **Mapping**: Leaflet.js with Leaflet Draw
- **Database**: PostgreSQL with Prisma ORM
- **Authentication**: bcryptjs
- **AI**: Google Gemini API

## Prerequisites

- Node.js 18+ installed
- PostgreSQL database
- npm or yarn package manager

## Installation

1. **Clone or navigate to your project directory**:

```bash
cd ecoweeder
```

2. **Install dependencies**:

```bash
npm install
```

3. **Set up environment variables**:

```bash
cp .env.example .env
```

Edit `.env` and add your database connection string:

```env
DATABASE_URL="postgresql://USER:PASSWORD@HOST:PORT/DATABASE?schema=public"
```

4. **Initialize Prisma and database**:

```bash
npm run prisma:generate
npm run prisma:push
```

5. **Run the development server**:

```bash
npm run dev
```

6. **Open your browser**:
   Navigate to [http://localhost:3000](http://localhost:3000)

## Project Structure

```
ecoweeder/
├── app/
│   ├── layout.tsx                    # Root layout
│   ├── globals.css                   # Global styles
│   ├── auth/
│   │   ├── login/page.tsx           # Login page
│   │   └── register/page.tsx        # Registration page
│   ├── dashboard/
│   │   ├── layout.tsx               # Dashboard layout with sidebar
│   │   ├── kml-generator/page.tsx   # KML generation page
│   │   ├── kml-viewer/page.tsx      # KML viewing page
│   │   ├── ai-agent/page.tsx        # AI navigation page
│   │   └── about/page.tsx           # About page
│   └── api/
│       └── auth/
│           ├── login/route.ts       # Login API endpoint
│           └── register/route.ts    # Registration API endpoint
├── components/
│   ├── layout/
│   │   ├── dashboard-sidebar.tsx    # Navigation sidebar
│   │   └── dashboard-header.tsx     # Dashboard header
│   ├── map/
│   │   ├── mapbox-draw-map-component.tsx      # KML generator map
│   │   ├── leaflet-kml-viewer-component.tsx   # KML viewer map
│   │   └── ai-agent-map-component.tsx         # AI navigation map
│   └── ui/                          # shadcn/ui components
│       ├── button.tsx
│       ├── input.tsx
│       ├── label.tsx
│       └── card.tsx
├── lib/
│   ├── utils.ts                     # Utility functions
│   ├── kml-converter.ts             # GeoJSON to KML converter
│   └── kml-parser.ts                # KML to GeoJSON parser
├── prisma/
│   └── schema.prisma                # Database schema
├── package.json                     # Dependencies
├── tailwind.config.js               # Tailwind configuration
├── next.config.js                   # Next.js configuration
├── tsconfig.json                    # TypeScript configuration
└── README.md                        # This file
```

## Usage

### 1. Authentication

- Create an account using the registration page
- Login with your credentials

### 2. KML Generator

- Navigate to "KML Downloader" from the sidebar
- Use map tools to draw shapes:
  - Polygon tool for custom areas
  - Rectangle tool for quick rectangular areas
  - Circle tool for circular zones
  - Line tool for paths
  - Marker tool for points of interest
- View GeoJSON output in real-time
- Click "Download KML File" to export

### 3. KML Viewer

- Navigate to "KML Opener" from the sidebar
- Click "Open KML File" to upload a KML file
- View your data visualized on the map
- Inspect loaded features in the sidebar

### 4. AI Navigation Agent

- Navigate to "AI Agent" from the sidebar
- Set your Gemini API key in the sidebar panel
- Load a KML file with waypoints
- Add a vehicle icon to the map
- Click "Start Navigation" to begin AI-powered routing
- The vehicle will automatically navigate through waypoints

## Map Layers Explained

### KML Generator

- **Layer 1**: OpenStreetMap base layer

### KML Viewer

- **Layer 1**: OpenStreetMap base layer
- **Layer 2**: Loaded KML data (blue markers/shapes)

### AI Agent

- **Layer 1**: OpenStreetMap base layer
- **Layer 2**: Loaded KML waypoints (red markers/shapes)
- **Layer 3**: AI-controlled vehicle (car emoji, movable)

## API Integration

### Gemini API Setup

1. Get your API key from [Google AI Studio](https://makersuite.google.com/app/apikey)
2. Navigate to "AI Agent" in the dashboard
3. Enter your API key in the sidebar
4. Click the save button
5. Your key will be stored locally for future sessions

## Database Management

View and manage database with Prisma Studio:

```bash
npm run prisma:studio
```

Reset database:

```bash
npx prisma migrate reset
```

## Troubleshooting

### Leaflet icons not showing

If map markers don't display correctly, ensure Leaflet CSS is properly loaded in your components.

### Database connection issues

- Verify DATABASE_URL in `.env`
- Ensure PostgreSQL is running
- Check database permissions

### Build errors

```bash
rm -rf .next node_modules
npm install
npm run dev
```

On Windows, Next.js can encounter errors when the project path contains spaces or special characters (for example `E:\droneworx ai website\...`). If you see errors referencing a protocol like `e:` or loader ESM URL scheme issues, move the project to a path without spaces (for example `C:\projects\ecoweeder`) or run the project inside WSL. After moving the folder, reinstall dependencies:

```bash
npm install --legacy-peer-deps
npm run dev
```

## Production Deployment

1. Build the application:

```bash
npm run build
```

2. Start production server:

```bash
npm start
```

### Environment Variables for Production

Ensure these are set:

- `DATABASE_URL`: Production database connection
- `NODE_ENV=production`

## Contributing

This is a private project. For questions or suggestions, contact the development team.

## License

Proprietary - All rights reserved

## Version

1.0.0 - Initial Release

---

Built with ❤️ using Next.js, TypeScript, and modern web technologies
