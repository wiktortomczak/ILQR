#!/usr/bin/env python

import sys

import cv2
import numpy as np
import pandas as pd
import skvideo.io


def main():
  CartPole(dt=None).render(
    CartPole.ReadTrajectoryFromCSV(sys.argv[1]))
  

class CartPole:
    def __init__(self, dt,
                 mc=1.0,
                 mp=0.1,
                 l=1.0,
                 g=9.80665):
        """CartPole problem.

        Args:
            dt: Time step [s].
            mc: Cart mass [kg].
            mp: Pendulum mass [kg].
            l: Pendulum length [m].
            g: Gravity acceleration [m/s^2].

        Note:
            state x: [px, px', theta, theta'].
            input u: [Fx], it's bound to (-1, 1) in `step`.
            theta: 0 is pointing up and increasing clockwise.
        """

        self.dt = dt
        self.mc = mc
        self.mp = mp
        self.l = l
        self.g = g

        self.action_dim = 1
        self.state_dim = 4
        # self.pi = tf.constant(np.pi)

    def step(self, x, u):
        """Calculates the next state."""
        x_ = x[..., 0]
        x_dot = x[..., 1]
        theta = x[..., 2]
        sin_theta = tf.sin(theta)
        cos_theta = tf.cos(theta)
        theta_dot = x[..., 3]
        F = tf.tanh(u[..., 0])

        # Define frictionless dynamics as per (Razvan V. Florian, 2007)
        # Paper: https://coneural.org/florian/papers/05_cart_pole.pdf

        # Eq. 23
        temp = (F + self.mp * self.l * theta_dot**2 * sin_theta) / (self.mc + self.mp)
        numerator = self.g * sin_theta - cos_theta * temp
        denominator = self.l * (4.0 / 3.0 - self.mp * cos_theta**2 / (self.mc + self.mp))
        theta_dot_dot = numerator / denominator

        # Eq. 24
        x_dot_dot = temp - self.mp * self.l * theta_dot_dot * cos_theta / (self.mc + self.mp)

        return tf.transpose(tf.stack([
            x_ + x_dot * self.dt,
            x_dot + x_dot_dot * self.dt,
            theta + theta_dot * self.dt,
            theta_dot + theta_dot_dot * self.dt,
        ]))

    def cost(self, x, u):
        """Calculates the state-input cost."""
        x_ = x[..., 0]
        x_dot = x[..., 1]
        # Shift by pi so zero angle, and hence the cost, is the up position
        cos_theta = tf.cos(x[..., 2] + self.pi)
        theta_dot = x[..., 3]

        # Note: There is no penalty for the input
        #       However, it's bounded to (-1, 1) in `step`
        g = 6 * x_**2 + 12 * (1 + cos_theta)**2 + 0.1 * x_dot**2 + 0.1 * theta_dot**2
        return g * self.dt

    def render(self, x_history, file_name='cartpole.mp4'):
        """Renders the whole trajectory to the mp4 file."""
        frames = []
        for x in x_history:
            frames.append(self._render_state(x))
        skvideo.io.vwrite(file_name, np.stack(frames))

    def _render_state(self, x):
        """Renders one frame."""
        screen_width = 640
        screen_height = 480
        world_width = 2 * 2.4
        scale = screen_width / world_width
        cart_y = screen_height - 100
        pole_width = 10
        pole_len = scale * self.l
        cart_width = 50
        cart_height = 30


        # Get the cart position and the pole angle
        position, _, angle, _ = x
        cart_x = int(position * scale + screen_width / 2.0)

        # Prepare a blank image of screen size
        img = np.ones((screen_height, screen_width, 3)) * 255.

        # Draw a rail
        img = cv2.line(
            img,
            pt1=(0, cart_y),
            pt2=(screen_width, cart_y),
            color=(0, 0, 0)
        )

        # Draw a cart
        img = cv2.rectangle(
            img,
            pt1=(cart_x - cart_width // 2,
                    cart_y - cart_height // 2),
            pt2=(cart_x + cart_width // 2,
                    cart_y + cart_height // 2),
            color=(0, 0, 0),
            thickness=cv2.FILLED
        )

        # Draw a pole
        img = cv2.line(
            img,
            pt1=(cart_x, cart_y),
            pt2=(int(cart_x + pole_len * np.sin(angle)),
                    int(cart_y - pole_len * np.cos(angle))),
            color=(204, 153, 102),
            thickness=pole_width
        )

        # Draw an axle
        img = cv2.circle(
            img,
            center=(cart_x, cart_y),
            radius=pole_width // 2,
            color=(127, 127, 204),
            thickness=cv2.FILLED
        )

        return img.astype(np.uint8)

    @classmethod
    def ReadTrajectoryFromCSV(cls, csv_path):
      return pd.read_csv(
        csv_path, sep=',\s*', usecols=['x', 'x_dot', 'theta', 'theta_dot']
      ).values


main()
