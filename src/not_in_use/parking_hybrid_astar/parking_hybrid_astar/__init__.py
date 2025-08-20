# parking_hybrid_astar/__init__.py
import os
import sys

# 현재 파일의 디렉토리 경로
current_dir = os.path.dirname(os.path.abspath(__file__))

# 프로젝트 루트 경로 추가
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)
    