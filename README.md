# Ecoweeder - KML Generator & AI Navigation Platform

A comprehensive Next.js application for KML file generation, visualization, and AI-powered navigation using Google Gemini API.

## Features

- 🗺️ **KML Generator**: Interactive map drawing with polygon, rectangle, circle, line, and marker tools
- 📍 **KML Viewer**: Load and visualize KML files on OpenStreetMap
- 🤖 **AI Navigation Agent**: AI-powered vehicle navigation through waypoints
- 🔐 **User Authentication**: Secure login and registration system
- 📱 **Responsive Design**: Modern UI built with Tailwind CSS and shadcn/ui

## Tech Stack

- **Framework**: Next.js 14 with App Router
- **Language**: TypeScript
- **Styling**: Tailwind CSS
- **UI Components**: shadcn/ui with Radix UI
- **Mapping**: Leaflet.js with Leaflet Draw
- **Database**: PostgreSQL with Prisma ORM
- **Authentication**: bcryptjs
- **AI**: Google Gemini API

## Installation

1. **Clone and install dependencies**:
   ```bash
   git clone https://github.com/your-repo/ecoweeder.git
   cd ecoweeder
   npm install
   ```

2. **Set up environment variables and database**:
   ```bash
   cp .env.example .env
   # Edit .env with your DATABASE_URL
   npm run prisma:generate
   npm run prisma:push
   ```

3. **Run the development server**:
   ```bash
   npm run dev
   ```

## Usage

- **Authentication**: Register and log in to access the dashboard.
- **KML Generator**: Draw map features and download them as a KML file.
- **KML Viewer**: Upload and visualize KML files.
- **AI Navigation Agent**: Set your Gemini API key, load a KML with waypoints, and start the AI-powered navigation.

## Production Deployment

```bash
npm run build
npm start
```

---

_Proprietary - All rights reserved_