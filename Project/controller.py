import pinocchio as pin
from os.path import dirname, join, abspath

def loadURDF():
    pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "Project/urdf")
    #print(pinocchio_model_dir)
    mesh_dir = pinocchio_model_dir
    urdf_model_path = join(pinocchio_model_dir,"robot.urdf")

    return pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

# if __name__ == '__main__':
#     model, collision_model, visual_model = loadURDF()
#     print(pin.neutral(model))
