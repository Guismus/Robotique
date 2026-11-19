"""Enregistrement du banc physique, sans effet sur les commandes."""

import cv2
import mujoco
import imageio_ffmpeg


class Recorder:
    def __init__(self, sim, path):
        self.renderer = mujoco.Renderer(sim.m, height=360, width=480)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0.12, -0.04, 0.07]
        self.camera.distance = 0.75
        self.camera.azimuth = 145
        self.camera.elevation = -28
        self.writer = imageio_ffmpeg.write_frames(
            str(path), (480, 360), fps=8, codec="libx264", output_params=["-crf", "23"]
        )
        self.writer.send(None)
        self.last = -1
        self.phase = "INITIALISATION"

    def frame(self, sim):
        if sim.d.time - self.last < 0.5:
            return
        self.last = sim.d.time
        self.renderer.update_scene(sim.d, camera=self.camera)
        frame = self.renderer.render().copy()
        cv2.putText(
            frame,
            self.phase,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            "Simulation - lecture x4",
            (12, 340),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        self.writer.send(frame)

    def finish(self, sim, path):
        self.renderer.update_scene(sim.d, camera=self.camera)
        cv2.imwrite(str(path), cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR))
        self.writer.close()
        self.renderer.close()


def final_image(sim, path):
    renderer = mujoco.Renderer(sim.m, height=480, width=640)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.07, -0.16, 0.08]
    camera.distance = 0.52
    camera.azimuth = 145
    camera.elevation = -25
    renderer.update_scene(sim.d, camera=camera)
    cv2.imwrite(str(path), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
    renderer.close()
