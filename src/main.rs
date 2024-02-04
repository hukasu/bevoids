mod alignment;
mod cohesion;
mod separation;

use std::ops::Neg;

use alignment::{Alignment, AlignmentPlugin};
use bevy::{
    app::{App, FixedUpdate, Startup},
    asset::{Assets, Handle},
    core_pipeline::core_2d::Camera2dBundle,
    ecs::{
        component::Component,
        reflect::ReflectResource,
        schedule::{IntoSystemConfigs, IntoSystemSetConfigs, SystemSet},
        system::{Commands, Local, Query, Res, ResMut, Resource},
    },
    math::{Quat, Vec3},
    reflect::Reflect,
    render::mesh::Mesh,
    sprite::{ColorMesh2dBundle, Mesh2dHandle},
    transform::components::Transform,
    DefaultPlugins,
};
use bevy_inspector_egui::{
    inspector_options::ReflectInspectorOptions, quick::ResourceInspectorPlugin, InspectorOptions,
};
use cohesion::{Cohesion, CohesionPlugin};
use separation::{Separation, SeparationPlugin};

#[derive(Debug, Clone, Hash, PartialEq, Eq, SystemSet)]
enum ForceCalculationSystemSet {
    Pre,
    ForceCalculation,
    Post,
}

#[derive(Debug, Default, Component)]
struct Boid {
    pub velocity: Vec3,
}

#[derive(Debug, Default, Resource, Reflect, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
pub struct BoidControl {
    #[inspector(min = 5.0)]
    max_speed: f32,
    #[inspector(min = 0.0)]
    vision_range: f32,
    #[inspector(min = 0.0)]
    separation_distance: f32,
    #[inspector(min = 0.0, max = 0.1)]
    separation_factor: f32,
    #[inspector(min = 0.0, max = 1.0)]
    alignment_factor: f32,
    #[inspector(min = 0.0, max = 0.1)]
    cohesion_factor: f32,
    #[inspector(min = 0.0, max = 1.0)]
    return_factor: f32,
}

fn main() {
    App::new()
        // Resources
        .insert_resource(BoidControl {
            max_speed: 5.,
            vision_range: 25.,
            separation_distance: 10.,
            separation_factor: 0.05,
            alignment_factor: 0.05,
            cohesion_factor: 0.005,
            return_factor: 0.001,
        })
        .register_type::<BoidControl>()
        // Plugins
        .add_plugins(DefaultPlugins)
        .add_plugins(SeparationPlugin)
        .add_plugins(AlignmentPlugin)
        .add_plugins(CohesionPlugin)
        // Inspector Plugins
        .add_plugins(ResourceInspectorPlugin::<BoidControl>::default())
        // SystemSet
        .configure_sets(
            FixedUpdate,
            (
                ForceCalculationSystemSet::Pre,
                ForceCalculationSystemSet::ForceCalculation,
                ForceCalculationSystemSet::Post,
            )
                .chain(),
        )
        // Systems
        .add_systems(Startup, spawn_camera)
        .add_systems(Startup, spawn_boids)
        .add_systems(
            FixedUpdate,
            apply_forces.in_set(ForceCalculationSystemSet::Post),
        )
        .run();
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

fn spawn_boids(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mesh_handle: Local<Handle<Mesh>>,
    control: Res<BoidControl>,
) {
    if !meshes.contains(&*mesh_handle) {
        *mesh_handle = meshes.add(bevy::render::mesh::shape::RegularPolygon::new(5., 3).into());
    }

    for _ in 0..2000 {
        commands.spawn((
            Boid {
                velocity: Vec3::new(
                    (rand::random::<f32>() - 0.5) * control.max_speed,
                    (rand::random::<f32>() - 0.5) * control.max_speed,
                    0.,
                ),
            },
            Separation::default(),
            Alignment::default(),
            Cohesion::default(),
            ColorMesh2dBundle {
                mesh: Mesh2dHandle(mesh_handle.clone()),
                transform: bevy::transform::components::Transform {
                    translation: Vec3::new(
                        (rand::random::<f32>() - 0.5) * 1000.,
                        (rand::random::<f32>() - 0.5) * 1000.,
                        0.,
                    ),
                    rotation: Quat::default(),
                    scale: Vec3::new(0.5, 1., 1.),
                },
                ..Default::default()
            },
        ));
    }
}

fn apply_forces(
    mut boids: Query<(
        &mut Boid,
        &mut Transform,
        &Separation,
        &Alignment,
        &Cohesion,
    )>,
    control: Res<BoidControl>,
) {
    for (mut boid, mut transform, separation, alignment, cohesion) in boids.iter_mut() {
        let return_force = transform.translation.normalize().neg()
            * transform.translation.length().log2()
            * control.return_factor;

        boid.velocity += separation.avoid_direction
            + alignment.neighborhood_alignment
            + cohesion.neighborhood_cohesion
            + return_force;

        if boid.velocity.length() >= control.max_speed {
            boid.velocity *= 0.995;
        } else if boid.velocity.length() < control.max_speed / 5. {
            boid.velocity *= 1.005;
        }

        let movement_direction = boid.velocity.normalize();
        transform.translation += boid.velocity;
        transform.rotation = Quat::from_rotation_arc(Vec3::Y, movement_direction);
    }
}
