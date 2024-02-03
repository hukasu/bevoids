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

pub struct AlignmentPlugin;

impl Plugin for AlignmentPlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        app
            // Systems
            .add_systems(
                FixedUpdate,
                clear_alignment.in_set(ForceCalculationSystemSet::Pre),
            )
            .add_systems(
                FixedUpdate,
                calculate_alignment.in_set(ForceCalculationSystemSet::ForceCalculation),
            );
    }
}

#[derive(Debug, Default, Component)]
pub struct Alignment {
    pub neighborhood_alignment: Vec3,
    pub neighbors: usize,
}

fn clear_alignment(mut boids: Query<&mut Alignment, With<Boid>>) {
    for mut boid in boids.iter_mut() {
        boid.neighborhood_alignment = Vec3::default();
        boid.neighbors = 0;
    }
}

fn calculate_alignment(
    mut boids: Query<(&mut Alignment, &Boid, &Transform)>,
    control: Res<BoidControl>,
) {
    let mut comb_iter = boids.iter_combinations_mut();
    while let Some([mut left, mut right]) = comb_iter.fetch_next() {
        let pos_diff = left.2.translation - right.2.translation;
        if pos_diff.length() < control.vision_range {
            left.0.neighbors += 1;
            left.0.neighborhood_alignment += right.1.velocity;

            right.0.neighbors += 1;
            right.0.neighborhood_alignment += left.1.velocity;
        }
    }

    for (mut alignment, boid, _) in boids.iter_mut() {
        if alignment.neighbors > 0 {
            let average_alignment = alignment.neighborhood_alignment / alignment.neighbors as f32;
            alignment.neighborhood_alignment =
                (average_alignment - boid.velocity) * control.alignment_factor;
        }
    }
}
