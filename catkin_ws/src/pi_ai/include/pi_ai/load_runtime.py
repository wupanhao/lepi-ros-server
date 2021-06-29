try:
    import importlib.util
except Exception as e:
    print(e)
    
def load_tflite_model(model_path,use_TPU = False):
    # Import TensorFlow libraries
    # If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
    # If using Coral Edge TPU, import the load_delegate library
    pkg = importlib.util.find_spec('tflite_runtime')
    if pkg:
        from tflite_runtime.interpreter import Interpreter
        if use_TPU:
            from tflite_runtime.interpreter import load_delegate
    else:
        from tensorflow.lite.python.interpreter import Interpreter
        if use_TPU:
            from tensorflow.lite.python.interpreter import load_delegate
    # Load the Tensorflow Lite model.
    # If using Edge TPU, use special load_delegate argument
    if use_TPU:
        interpreter = Interpreter(model_path=model_path,
                                experimental_delegates=[load_delegate('libedgetpu.so.1',{})])
    else:
        interpreter = Interpreter(model_path=model_path)
    print(model_path)
    interpreter.allocate_tensors()
    return interpreter