import xml.etree.ElementTree as ET
tree = ET.parse('/home/developer/multipanda_ws/install/franka_description/share/franka_description/meshes/visual/link4.dae')
root = tree.getroot()
ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
for node in root.findall('.//c:node', ns):
    print("Node:", node.get('id', node.get('name')))
    matrix = node.find('c:matrix', ns)
    if matrix is not None:
        print("Matrix:", matrix.text)
    translate = node.find('c:translate', ns)
    if translate is not None:
        print("Translate:", translate.text)
    rotate = node.findall('c:rotate', ns)
    for r in rotate:
        print("Rotate:", r.text)
