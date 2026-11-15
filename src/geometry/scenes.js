import { Vec2 } from '../core/vec2.js';
import { Body } from '../physics/body.js';
import { World } from '../physics/world.js';
import { Solver } from '../physics/solver.js';
import { BoxShape, WedgeShape, RotatedConvex } from './shapes.js';

// Static floor is a wide thin box so every interaction uses the same
// GJK/EPA/manifold pipeline as prop-on-prop contact.
function makeFloor(world) {
  const floor = new Body(new BoxShape(8, 0.5), {
    static: true,
    position: new Vec2(0, -3),
    label: 'floor',
    friction: 0.6
  });
  world.addBody(floor);
  return floor;
}

export function buildScene(name) {
  const world = new World({ recordTrail: true });
  const solver = new Solver(world);
  const scene = { name, world, solver, grab: null };
  const add = (body) => {
    world.addBody(body);
    return body;
  };

  switch (name) {
    case 'stack': {
      makeFloor(world);
      add(new Body(new BoxShape(0.6, 0.6), { position: new Vec2(0, -2.0), label: 'box-1' }));
      add(new Body(new BoxShape(0.5, 0.5), { position: new Vec2(0.02, -0.85), angle: 0.15, label: 'box-2' }));
      add(new Body(new WedgeShape(0.55, 0.55), { position: new Vec2(-0.05, 0.25), label: 'wedge-1' }));
      break;
    }
    case 'wedge': {
      makeFloor(world);
      add(new Body(new WedgeShape(0.8, 0.8), { static: true, position: new Vec2(-2.2, -1.7), label: 'ramp' }));
      add(new Body(new BoxShape(0.4, 0.4), { position: new Vec2(-2.6, -0.4), label: 'slider' }));
      add(new Body(new BoxShape(0.5, 0.35), { position: new Vec2(0.8, -1.2), angle: 0.4, label: 'box-side' }));
      break;
    }
    case 'rotated': {
      makeFloor(world);
      const hex = new RotatedConvex(
        [
          new Vec2(0.6, 0), new Vec2(0.3, 0.52), new Vec2(-0.3, 0.52),
          new Vec2(-0.6, 0), new Vec2(-0.3, -0.52), new Vec2(0.3, -0.52)
        ],
        'hex-prop'
      );
      add(new Body(hex, { position: new Vec2(0.5, -1.2), angle: 0.3, label: 'hex' }));
      add(new Body(new BoxShape(0.45, 0.45), { position: new Vec2(-1.2, -1.4), angle: -0.6, label: 'tilted-box' }));
      add(new Body(new WedgeShape(0.5, 0.5), { position: new Vec2(1.8, -1.6), angle: 1.2, label: 'wedge-spin' }));
      break;
    }
    case 'degenerate': {
      makeFloor(world);
      // Near-coplanar edge-edge resting contact and shared-height vertices.
      add(new Body(new BoxShape(0.7, 0.5), { position: new Vec2(-0.7, -2.0), angle: 0.003, label: 'edge-a' }));
      add(new Body(new BoxShape(0.7, 0.5), { position: new Vec2(0.72, -2.0), angle: -0.002, label: 'edge-b' }));
      // Pentagon built from input containing duplicate and collinear points.
      const noisy = new RotatedConvex(
        [
          new Vec2(0, 0.5), new Vec2(0, 0.5),
          new Vec2(0.48, 0.16), new Vec2(0.3, -0.4),
          new Vec2(0.15, -0.4), new Vec2(-0.3, -0.4), new Vec2(-0.48, 0.16)
        ],
        'noisy-pentagon'
      );
      add(new Body(noisy, { position: new Vec2(0, -0.8), label: 'pentagon' }));
      break;
    }
    case 'push':
    default: {
      makeFloor(world);
      add(new Body(new BoxShape(0.5, 0.5), { position: new Vec2(-1.6, -1.9), label: 'pusher' }));
      add(new Body(new BoxShape(0.55, 0.55), { position: new Vec2(0, -1.95), label: 'crate' }));
      add(new Body(new WedgeShape(0.6, 0.6), { position: new Vec2(1.5, -1.9), label: 'wedge-stop' }));
      break;
    }
  }
  return scene;
}

export const SCENE_NAMES = ['push', 'stack', 'wedge', 'rotated', 'degenerate'];
