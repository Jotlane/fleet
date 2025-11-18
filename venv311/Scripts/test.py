from dronekit import connect
import collections, collections.abc
for _n in ("MutableMapping","Mapping","Sequence","Iterable"):
    if not hasattr(collections,_n):
        setattr(collections,_n,getattr(collections.abc,_n))
v = connect("udpin:127.0.0.1:15023", wait_ready=True)
print(v.version)
v.close()