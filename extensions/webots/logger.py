import math
import json


class MoveResultLogger:
    """
    Логгер точности укладки объектов.
    - objects_plan: [{ 'def': str, 'start_xyz': [x,y,z], 'end_xyz': [x,y,z] }, ...]
    - place_done_markers: [{ 'traj_index': int, 'object_idx': int }, ...]
    - results_path: путь к JSON, куда писать результаты
    - robot: Supervisor (для getFromDef)
    - robot_node: узел робота (если нужна трансформация в локальные координаты)
    - transform_fn: опционально функция transform(world_xyz) -> xyz (например, world->local)
    """
    def __init__(self, *, robot, robot_node, objects_plan, place_done_markers,
                 results_path, transform_fn=None):
        self.robot = robot
        self.robot_node = robot_node
        self.objects_plan = objects_plan or []
        self.place_done_markers = place_done_markers or []
        self.results_path = results_path
        self.transform_fn = transform_fn
        self.next_marker_idx = 0
        self.results = []
        # кэш узлов по DEF
        self._node_cache = {}

    @staticmethod
    def _euclid(a, b):
        return float(((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2) ** 0.5)

    def _get_node(self, def_name):
        node = self._node_cache.get(def_name)
        if node is None:
            node = self.robot.getFromDef(def_name)
            self._node_cache[def_name] = node
        return node

    def try_log(self, traj_index_reached: int):
        """
        Вызывать каждый раз, когда индекс траектории продвинулся вперёд.
        Если достигнут очередной маркер — логируем результат для соответствующего объекта.
        """
        if self.next_marker_idx >= len(self.place_done_markers):
            return

        marker = self.place_done_markers[self.next_marker_idx]
        if traj_index_reached < marker["traj_index"]:
            return

        obj_i = marker["object_idx"]
        if not (0 <= obj_i < len(self.objects_plan)):
            self.next_marker_idx += 1
            return

        plan = self.objects_plan[obj_i]
        def_name = plan["def"]
        end_xyz  = plan["end_xyz"]
        start_xyz = plan["start_xyz"]

        node = self._get_node(def_name)
        if node is None:
            print(f"[WARN] Узел DEF '{def_name}' не найден для измерения.")
            self.next_marker_idx += 1
            return

        world_pos = node.getPosition()  # [x,y,z] в мировых координатах
        cur_xyz = [float(world_pos[0]), float(world_pos[1]), float(world_pos[2])]

        # При необходимости перевести в локальные координаты:
        if self.transform_fn is not None:
            try:
                cur_xyz = list(map(float, self.transform_fn(cur_xyz)))
            except Exception as e:
                print(f"[WARN] Трансформация координат для '{def_name}' провалилась: {e}")

        err = self._euclid(cur_xyz, end_xyz)

        self.results.append({
            "object": def_name,
            "start_pose": start_xyz,
            "end_pose": end_xyz,
            "current_pose": cur_xyz,
            "euclidian_error": err
        })

        try:
            with open(self.results_path, "w", encoding="utf-8") as f:
                json.dump(self.results, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"[WARN] Ошибка записи {self.results_path}: {e}")

        self.next_marker_idx += 1

    def finalize(self):
        """Финальное сохранение."""
        try:
            with open(self.results_path, "w", encoding="utf-8") as f:
                json.dump(self.results, f, ensure_ascii=False, indent=2)
            print(f"[INFO] Результаты перемещений сохранены в {self.results_path}")
        except Exception as e:
            print(f"[WARN] Ошибка записи {self.results_path}: {e}")


class InitialObjectsSnapshotLogger:
    """
    Отдельный простой логгер, который делает только стартовый снимок объектов
    и пишет его в JSON. Вызывать ДО основного цикла.
    """
    def __init__(self, *, robot, robot_node, object_defs, transform_fn=None, outfile_path=None):
        self.robot = robot
        self.robot_node = robot_node
        self.object_defs = list(object_defs) if object_defs else []
        self.transform_fn = transform_fn
        self.outfile_path = outfile_path

    @staticmethod
    def _mat3_from_flat9(v):
        return [[v[0], v[1], v[2]],
                [v[3], v[4], v[5]],
                [v[6], v[7], v[8]]]

    @staticmethod
    def _mat3_T(M):
        return [[M[j][i] for j in range(3)] for i in range(3)]

    @staticmethod
    def _mat3_mul(A, B):
        return [
            [A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0],
             A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1],
             A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2]],
            [A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0],
             A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1],
             A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2]],
            [A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0],
             A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1],
             A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2]],
        ]

    @staticmethod
    def _rmat_to_rpy_zyx(R):
        sy = -R[2][0]
        cy = math.sqrt(max(0.0, 1.0 - sy*sy))
        if cy > 1e-8:
            roll  = math.atan2(R[2][1], R[2][2])
            pitch = math.asin(sy)
            yaw   = math.atan2(R[1][0], R[0][0])
        else:
            roll  = math.atan2(-R[1][2], R[1][1])
            pitch = math.asin(sy)
            yaw   = 0.0
        return (roll, pitch, yaw)

    def _get_node_color_safe(self, node):
        try:
            children = node.getField("children") if node.getField("children") else None
            if children and children.getCount() > 0:
                shape = children.getMFNode(0)
                if shape:
                    app_field = shape.getField("appearance")
                    if app_field:
                        app = app_field.getSFNode()
                        if app:
                            base = app.getField("baseColor")
                            if base:
                                v = base.getSFColor()
                                return [float(v[0]), float(v[1]), float(v[2])]
                            mat_field = app.getField("material")
                            if mat_field:
                                mat = mat_field.getSFNode()
                                if mat:
                                    diff = mat.getField("diffuseColor")
                                    if diff:
                                        v = diff.getSFColor()
                                        return [float(v[0]), float(v[1]), float(v[2])]
            app_field = node.getField("appearance")
            if app_field:
                app = app_field.getSFNode()
                if app:
                    base = app.getField("baseColor")
                    if base:
                        v = base.getSFColor()
                        return [float(v[0]), float(v[1]), float(v[2])]
                    mat_field = app.getField("material")
                    if mat_field:
                        mat = mat_field.getSFNode()
                        if mat:
                            diff = mat.getField("diffuseColor")
                            if diff:
                                v = diff.getSFColor()
                                return [float(v[0]), float(v[1]), float(v[2])]
        except Exception:
            pass
        return None

    def snapshot(self):
        if not self.object_defs:
            print("[WARN] InitialObjectsSnapshotLogger: object_defs пуст.")
            return []

        R_wr = self._mat3_from_flat9(self.robot_node.getOrientation())
        R_rw = self._mat3_T(R_wr)

        export = []
        for def_name in self.object_defs:
            node = self.robot.getFromDef(def_name)
            if node is None:
                export.append({
                    "def": def_name,
                    "position_robot": None,
                    "rpy_robot": None,
                    "color": None,
                    "note": "node not found"
                })
                continue

            wp = node.getPosition()
            pos_world = [float(wp[0]), float(wp[1]), float(wp[2])]
            try:
                pos_robot = list(map(float, self.transform_fn(pos_world))) if self.transform_fn else pos_world
            except Exception:
                pos_robot = pos_world

            R_wo = self._mat3_from_flat9(node.getOrientation())
            R_ro = self._mat3_mul(R_rw, R_wo)
            rpy_robot = self._rmat_to_rpy_zyx(R_ro)

            color = self._get_node_color_safe(node)

            export.append({
                "def": def_name,
                "position_robot": pos_robot,
                "rpy_robot": [float(rpy_robot[0]), float(rpy_robot[1]), float(rpy_robot[2])],
                "color": color
            })

        if self.outfile_path:
            try:
                with open(self.outfile_path, "w", encoding="utf-8") as f:
                    json.dump(export, f, ensure_ascii=False, indent=2)
                print(f"[INFO] Стартовые параметры объектов сохранены в {self.outfile_path}")
            except Exception as e:
                print(f"[WARN] Не удалось записать стартовый JSON '{self.outfile_path}': {e}")
        return export
