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
