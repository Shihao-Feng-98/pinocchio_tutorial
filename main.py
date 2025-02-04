import numpy as np
from scipy.spatial.transform import Rotation
import pinocchio as pin

def test01():
    model = pin.buildModelsFromUrdf("./ur5/urdf/ur5_robot_copy.urdf")[0] # FIXME: can not use relative path in urdf
    print(f"model name: {model.name}")
    data = model.createData()
    
    q = pin.randomConfiguration(model)
    print(f"q: {q}")
    pin.forwardKinematics(model, data, q)
    for name, oMi in zip(model.names, data.oMi):
        print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))

    v = np.zeros(model.nv)
    a = np.zeros(model.nv)
    tau = pin.rnea(model, data, q, v, a)
    print(f"tau: {tau}\n\n\n")


def test02():
    # skew
    p1 = np.array([0,0,1])
    M1 = pin.skew(p1)
    print(f"Vector: {p1.T}")
    print(f"Skew symmetric matrix: \n{M1}\n")

    # so3 -> SO3
    w1 = np.array([1,0,0])
    R1 = pin.exp3(w1)
    print(f"Rotation axis: {w1}")
    print(f"Rotation matrix: \n{R1}\n")

    # se3 -> SE3
    v = np.array([0,0,1])
    w = np.array([1,0,0])
    nu = pin.Motion(v, w)
    print(f"Motion experssion: \n{nu}")
    print(f"Linear term: {nu.linear}")
    print(f"Angular term: {nu.angular}")
    T = pin.exp6(nu)
    print(f"SE3 expression: \n{T}")
    print(f"Homogeneous expression: \n{T.homogeneous}")
    print(f"Rotation term: \n{T.rotation}")
    print(f"Translation term: {T.translation}\n")

    # SO3 -> so3
    R2 = Rotation.random()
    w2 = pin.log3(R2.as_matrix())
    print(f"Rotation axis: {w2}\n")

    # SE3 -> se3
    p2 = np.array([1,1,1])
    T2 = pin.SE3(R2.as_matrix(), p2)
    nu2 = pin.log6(T2)
    print(f"Motion: \n{nu2}")

    # spatial force
    f = np.array([1,0,0])
    tau = np.array([1,1,1])
    phi = pin.Force(f, tau)
    print(f"Force expression: \n{phi}")
    print(f"Linear term: {phi.linear}")
    print(f"Angular term: {phi.angular}\n")

    # adjoint
    T3 = pin.SE3.Random()
    adT = T3.toActionMatrix()
    print(f"SE3 expression: \n{T3}")
    print(f"Adjoint expression: \n{adT}")

if __name__ == "__main__":
    test01()
    test02()