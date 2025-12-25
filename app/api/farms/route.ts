import { NextRequest, NextResponse } from 'next/server';
import prisma from '@/lib/prisma';

export async function POST(request: NextRequest) {
  try {
    const body = await request.json();
    const { name, boundary } = body;

    // In a real app, you'd get the userId from the session
    // For now, we'll use a placeholder or find the first user
    const user = await prisma.user.findFirst();
    
    if (!user) {
      return NextResponse.json({ error: 'No user found' }, { status: 404 });
    }

    const farm = await prisma.farm.create({
      data: {
        name,
        boundary,
        userId: user.id
      }
    });

    return NextResponse.json(farm);
  } catch (error) {
    console.error('Error creating farm:', error);
    return NextResponse.json({ error: 'Internal Server Error' }, { status: 500 });
  }
}

export async function GET() {
  try {
    const farms = await prisma.farm.findMany({
      include: {
        missions: true
      }
    });
    return NextResponse.json(farms);
  } catch (error) {
    console.error('Error fetching farms:', error);
    return NextResponse.json({ error: 'Internal Server Error' }, { status: 500 });
  }
}
