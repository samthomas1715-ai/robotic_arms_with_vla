import cv2
import torch
import numpy as np
from pathlib import Path

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors



MODEL_PATH = Path(
    "/home/deepashikha/lerobot/act_two_arms_50000/"
    "checkpoints/050000/pretrained_model"
)

CAM_HIGH_INDEX = 8     
CAM_GRIPPER_INDEX = 4   
NUM_JOINTS = 6

if torch.cuda.is_available():
    DEVICE = "cuda"
    print("CUDA works... Using GPU")
else:
    DEVICE = "cpu"
    print("Using CPU")


torch.backends.cudnn.enabled = False

def load_model():
    print("Loading trained two-camera ACT model...")
    print("Checkpoint:", MODEL_PATH)
    print("Device:", DEVICE)

    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"Model checkpoint not found: {MODEL_PATH}"
        )

    policy = ACTPolicy.from_pretrained(str(MODEL_PATH))
    policy = policy.to(DEVICE)
    policy.eval()

    preprocessor, postprocessor = make_pre_post_processors(
        policy.config,
        pretrained_path=str(MODEL_PATH),
    )

    print("ACT model loaded successfully.")

    return policy, preprocessor, postprocessor



def open_camera(camera_index, camera_name):
    print(f"Opening {camera_name} at index {camera_index}...")

    camera = cv2.VideoCapture(camera_index)

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not camera.isOpened():
        camera.release()
        raise RuntimeError(
            f"Cannot open {camera_name} at index {camera_index}. "
            "Check the camera connection and index."
        )

    print(f"{camera_name} opened successfully.")
    return camera




@torch.inference_mode()
def predict_action(
    policy,
    preprocessor,
    postprocessor,
    high_image,
    gripper_image,
    state,
):
   
    high_tensor = torch.from_numpy(high_image.copy())
    high_tensor = high_tensor.permute(2, 0, 1).unsqueeze(0)
    high_tensor = high_tensor.to(torch.float32) / 255.0

    gripper_tensor = torch.from_numpy(gripper_image.copy())
    gripper_tensor = gripper_tensor.permute(2, 0, 1).unsqueeze(0)
    gripper_tensor = gripper_tensor.to(torch.float32) / 255.0

    
    state_tensor = torch.tensor(
        state,
        dtype=torch.float32,
    ).unsqueeze(0)

   
    observation = {
        "observation.images.cam_high": high_tensor,
        "observation.images.cam_gripper": gripper_tensor,
        "observation.state": state_tensor,
    }

   
    batch = preprocessor(observation)

    
    action = policy.select_action(batch)

   
    action = postprocessor(action)

    return action.squeeze(0).detach().cpu().numpy()




def main():
    policy, preprocessor, postprocessor = load_model()

    camera_high = None
    camera_gripper = None

    try:
        camera_high = open_camera(
            CAM_HIGH_INDEX, "High camera"
        )

        camera_gripper = open_camera(
            CAM_GRIPPER_INDEX, "Gripper camera"
        )

       
        test_state = np.zeros(NUM_JOINTS, dtype=np.float32)

        print("\nStarting two-camera inference test.")
        print("No motor commands will be sent.")
        print("Press Q in either camera window to exit.\n")

        while True:
            ret_high, frame_high = camera_high.read()
            ret_gripper, frame_gripper = camera_gripper.read()

            if not ret_high or not ret_gripper:
                print("Failed to read one or both camera frames.")
                break

           
            frame_high = cv2.resize(frame_high, (640, 480))
            frame_gripper = cv2.resize(
                frame_gripper, (640, 480)
            )

          
            rgb_high = cv2.cvtColor(
                frame_high, cv2.COLOR_BGR2RGB
            )

            rgb_gripper = cv2.cvtColor(
                frame_gripper, cv2.COLOR_BGR2RGB
            )

            predicted_action = predict_action(
                policy,
                preprocessor,
                postprocessor,
                rgb_high,
                rgb_gripper,
                test_state,
            )

            print("Predicted action:", predicted_action)

            cv2.imshow("High Camera", frame_high)
            cv2.imshow("Gripper Camera", frame_gripper)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    finally:
        if camera_high is not None:
            camera_high.release()

        if camera_gripper is not None:
            camera_gripper.release()

        cv2.destroyAllWindows()
        print("Two-camera inference test stopped.")


if __name__ == "__main__":
    main()
