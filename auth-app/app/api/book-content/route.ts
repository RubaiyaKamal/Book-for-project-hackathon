import { readFileSync } from 'fs';
import { join } from 'path';
import { NextResponse } from 'next/server';

export async function GET(request: Request) {
    const { searchParams } = new URL(request.url);
    const id = searchParams.get('id');

    if (!id) {
        return NextResponse.json({ error: 'Missing id parameter' }, { status: 400 });
    }

    // Map IDs to file paths
    const filePathMap: Record<string, string> = {
        'intro': 'intro.md',
        'intro-physical-ai': 'module-1/week-1-2/intro-physical-ai.md',
        'embodied-intelligence': 'module-1/week-1-2/embodied-intelligence.md',
        'humanoid-landscape': 'module-1/week-1-2/humanoid-landscape.md',
        'sensor-systems': 'module-1/week-1-2/sensor-systems.md',
        'ros2-architecture': 'module-1/week-3-5/ros2-architecture.md',
        'nodes-topics-services': 'module-1/week-3-5/nodes-topics-services.md',
        'building-packages': 'module-1/week-3-5/building-packages.md',
        'launch-files': 'module-1/week-3-5/launch-files.md',
        'gazebo-setup': 'module-2/week-6-7/gazebo-setup.md',
        'urdf-sdf': 'module-2/week-6-7/urdf-sdf.md',
        'physics-sensors': 'module-2/week-6-7/physics-sensors.md',
        'unity-visualization': 'module-2/week-6-7/unity-visualization.md',
        'isaac-sdk': 'module-3/week-8-10/isaac-sdk.md',
        'ai-perception': 'module-3/week-8-10/ai-perception.md',
        'reinforcement-learning': 'module-3/week-8-10/reinforcement-learning.md',
        'sim-to-real': 'module-3/week-8-10/sim-to-real.md',
        'humanoid-kinematics': 'module-4/week-11-12/humanoid-kinematics.md',
        'bipedal-locomotion': 'module-4/week-11-12/bipedal-locomotion.md',
        'manipulation-grasping': 'module-4/week-11-12/manipulation-grasping.md',
        'human-robot-interaction': 'module-4/week-11-12/human-robot-interaction.md',
        'gpt-integration': 'module-4/week-13/gpt-integration.md',
        'speech-recognition': 'module-4/week-13/speech-recognition.md',
        'multimodal-interaction': 'module-4/week-13/multimodal-interaction.md',
        'hardware-requirements': 'hardware/hardware-requirements.md',
        'assessments': 'assessments.md',
    };

    const filePath = filePathMap[id];
    if (!filePath) {
        return NextResponse.json({ error: 'Content not found' }, { status: 404 });
    }

    try {
        const fullPath = join(process.cwd(), 'docs', filePath);
        const content = readFileSync(fullPath, 'utf-8');

        // Extract title and sections
        const lines = content.split('\n');
        let title = id;
        const sections: string[] = [];

        for (const line of lines) {
            if (line.startsWith('title:')) {
                title = line.replace('title:', '').trim().replace(/['"]/g, '');
            }
            if (line.match(/^##\s+/)) {
                sections.push(line.replace(/^##\s+/, '').trim());
            }
        }

        return NextResponse.json({ title, content, sections });
    } catch (error) {
        return NextResponse.json({ error: 'Failed to read file' }, { status: 500 });
    }
}
