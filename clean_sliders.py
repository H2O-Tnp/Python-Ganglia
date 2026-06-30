import re

for fname in ['templates/index.html', 'templates/sim_index.html']:
    with open(fname, 'r', encoding='utf-8') as f:
        content = f.read()
    
    content = content.replace("'valPos'", "'numPos'")
    content = content.replace("'valVel'", "'numVel'")
    content = content.replace("'valCur'", "'numCur'")
    
    with open(fname, 'w', encoding='utf-8') as f:
        f.write(content)

print('Done fixing IDs!')
