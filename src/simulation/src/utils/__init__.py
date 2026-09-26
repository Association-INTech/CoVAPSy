from .plot_model_io import PlotModelIO
import onnxruntime as ort
import numpy as np

__all__ = ["PlotModelIO"]

def run_onnx_model(session: ort.InferenceSession, x: np.ndarray):
    return session.run(None, {"input": x})[0]