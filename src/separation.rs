use std::ops::Neg;

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

pub struct SeparationPlugin;

impl Plugin for SeparationPlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app
            // Systems
            .add_systems(
                FixedUpdate,
                clear_separation.in_set(ForceCalculationSystemSet::Pre),
            )
            .add_systems(
                FixedUpdate,
                calculate_separation.in_set(ForceCalculationSystemSet::ForceCalculation),
            );
    }
}

#[derive(Debug, Default, Component)]
pub struct Separation {
    pub avoid_direction: Vec3,
}

fn clear_separation(mut boids: Query<&mut Separation, With<Boid>>) {
    for mut boid in boids.iter_mut() {
        boid.avoid_direction = Vec3::default();
    }
}

fn calculate_separation(
    mut boids: Query<(&mut Separation, &Transform), With<Boid>>,
    control: Res<BoidControl>,
) {
    let mut comb_iter = boids.iter_combinations_mut();
    while let Some([mut left, mut right]) = comb_iter.fetch_next() {
        let pos_diff = left.1.translation - right.1.translation;
        let pos_dist = pos_diff.length();
        if pos_dist < control.separation_distance {
            left.0.avoid_direction +=
                pos_diff.normalize() * (control.separation_distance - pos_dist);
            right.0.avoid_direction +=
                pos_diff.neg().normalize() * (control.separation_distance - pos_dist);
        }
    }

    for (mut separation, _) in boids.iter_mut() {
        separation.avoid_direction *= control.separation_factor;
    }
}
