import fcl

objs1 = [fcl.CollisionObject(fcl.Sphere(0.2)), fcl.CollisionObject(fcl.Sphere(0.2))]
objs2 = [fcl.CollisionObject(fcl.Sphere(0.2)), fcl.CollisionObject(fcl.Sphere(0.2))]

manager1 = fcl.DynamicAABBTreeCollisionManager()
manager2 = fcl.DynamicAABBTreeCollisionManager()

manager1.registerObjects(objs1)
manager2.registerObjects(objs2)

manager1.setup()
manager2.setup()

# =====================================================================
# Managed internal (sub-n^2) collision checking
# =====================================================================
cdata = fcl.CollisionData()
manager1.collide(cdata, fcl.defaultCollisionCallback)
print('Collision within manager 1?: {}'.format(cdata.result.is_collision))

# =====================================================================
# Managed internal (sub-n^2) distance checking
# =====================================================================
ddata = fcl.DistanceData()
manager1.distance(ddata, fcl.defaultDistanceCallback)
print('Closest distance within manager 1?: {}'.format(ddata.result.min_distance))

# =====================================================================
# Managed one to many collision checking
# =====================================================================
req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
rdata = fcl.CollisionData(request=req)

manager1.collide(fcl.CollisionObject(fcl.Sphere(0.2)), rdata, fcl.defaultCollisionCallback)
print('Collision between manager 1 and Mesh?: {}'.format(rdata.result.is_collision))
print('Contacts:')
for c in rdata.result.contacts:
    print('\tO1: {}, O2: {}'.format(c.o1, c.o2))

    # =====================================================================
    # Managed many to many collision checking
    # =====================================================================
    rdata = fcl.CollisionData(request=req)
    manager1.collide(manager2, rdata, fcl.defaultCollisionCallback)
    print('Collision between manager 1 and manager 2?: {}'.format(rdata.result.is_collision))
    print('Contacts:')
    for c in rdata.result.contacts:
        print('\tO1: {}, O2: {}'.format(c.o1, c.o2))
