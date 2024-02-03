use bevy::{
    app::{FixedUpdate, Plugin},
    ecs::{
        component::Component,
        query::With,
        schedule::IntoSystemConfigs,
        system::{Query, Res},
    },
    math::Vec3,
    transform::components::Transform,
};

use crate::{Boid, BoidControl, ForceCalculationSystemSet};

pub struct CohesionPlugin;

impl Plugin for CohesionPlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app
            // Systems
            .add_systems(
                FixedUpdate,
                clear_cohesion.in_set(ForceCalculationSystemSet::Pre),
            )
            .add_systems(
                FixedUpdate,
                calculate_cohesion.in_set(ForceCalculationSystemSet::ForceCalculation),
            );
    }
}

#[derive(Debug, Default, Component)]
pub struct Cohesion {
    pub neighborhood_cohesion: Vec3,
    pub neighbors: usize,
}

fn clear_cohesion(mut boids: Query<&mut Cohesion, With<Boid>>) {
    for mut boid in boids.iter_mut() {
        boid.neighborhood_cohesion = Vec3::default();
        boid.neighbors = 0;
    }
}

fn calculate_cohesion(
    mut boids: Query<(&mut Cohesion, &Transform), With<Boid>>,
    control: Res<BoidControl>,
) {
    let mut comb_iter = boids.iter_combinations_mut();
    while let Some([mut left, mut right]) = comb_iter.fetch_next() {
        let pos_diff = left.1.translation - right.1.translation;
        if pos_diff.length() < control.vision_range {
            left.0.neighbors += 1;
            left.0.neighborhood_cohesion += right.1.translation;

            right.0.neighbors += 1;
            right.0.neighborhood_cohesion += left.1.translation;
        }
    }

    for (mut cohesion, transform) in boids.iter_mut() {
        if cohesion.neighbors > 0 {
            let average_position = cohesion.neighborhood_cohesion / cohesion.neighbors as f32;
            cohesion.neighborhood_cohesion =
                (average_position - transform.translation) * control.cohesion_factor;
        }
    }
}
