import tarfile
import os
import io

"""
tar = tarfile.open("asdf.asdf", "r:")
for member in tar.getmembers():
    print(member)
    if member.isfile():
        f = tar.extractfile(member)
        print(f.read())
tar.close()
"""
tar = tarfile.open("asdf.tar", "w")
file = "pip3 install snips-nlu"
s, tarinfo = io.BytesIO(file.encode("utf-8")), tarfile.TarInfo(name="snips2")
tarinfo.size  = len(file)
tar.addfile(tarinfo, fileobj=s)
tar.close()