from controller import Supervisor

class ObstacleSpawner:
    def __init__(self, supervisor: Supervisor):
        self.supervisor = supervisor
        self.timestep = int(supervisor.getBasicTimeStep())
        self.root_children = supervisor.getRoot().getField("children")
        self._counter = 0

    def _next_def(self, prefix: str) -> str:
        self._counter += 1
        return f"{prefix}_{self._counter}"

    @staticmethod
    def _color_tuple(color):
        r, g, b = color
        if max(r, g, b) > 1.0: 
            r, g, b = [c / 255.0 for c in (r, g, b)]
        return r, g, b

    @staticmethod
    def _to_rotation(rotation):
        if rotation is None:
            return "0 1 0 0"
        rx, ry, rz, angle = rotation
        return f"{rx} {ry} {rz} {angle}"

    def _insert_and_flush(self, node_string: str):
        self.root_children.importMFNodeFromString(-1, node_string)
        for _ in range(2):
            self.supervisor.step(self.timestep)

    def _spawn_solid(
        self,
        *,
        def_name: str,
        translation: tuple,
        rotation: str,
        color: tuple,
        transparency: float,
        geometry_block: str,
        bounding_block: str,
        static: bool,
        mass: float,
    ) -> str:
        x, y, z = translation
        r, g, b = self._color_tuple(color)
        locked = "TRUE" if static else "FALSE"

        node = f"""
DEF {def_name} Solid {{
  translation {x} {y} {z}
  rotation {rotation}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor {r} {g} {b}
          transparency {transparency}
        }}
      }}
      geometry {geometry_block}
    }}
  ]
  boundingObject {bounding_block}
  locked {locked}
  physics Physics {{
    mass {mass}
  }}
}}
"""
        self._insert_and_flush(node)
        return def_name


    def spawn_box(
        self,
        translation: tuple,
        size: tuple,
        *,
        name: str | None = None,
        color=(200, 60, 60),
        static=True,
        mass=5.0,
        rotation=None,
        transparency=0.0,
    ) -> str:
        """
        translation: (x, y, z) в м — позиция центра
        size: (sx, sy, sz) в м — габариты
        name: кастомное имя DEF; если None — сгенерируется 'BOX_i'
        """
        sx, sy, sz = size
        def_name = name if name else self._next_def("BOX")
        rot_str = self._to_rotation(rotation)

        geometry = f"Box {{ size {sx} {sy} {sz} }}"
        bounding = f"Box {{ size {sx} {sy} {sz} }}"

        return self._spawn_solid(
            def_name=def_name,
            translation=translation,
            rotation=rot_str,
            color=color,
            transparency=transparency,
            geometry_block=geometry,
            bounding_block=bounding,
            static=static,
            mass=mass,
        )

    def spawn_cylinder(
        self,
        translation: tuple,
        radius: float,
        height: float,
        *,
        name: str | None = None,
        color=(60, 120, 220),
        static=True,
        mass=5.0,
        rotation=None,
        transparency=0.0,
    ) -> str:
        """
        translation: (x, y, z) в м — позиция центра
        radius: м
        height: м — ось цилиндра совпадает с осью Y
        name: кастомное имя DEF; если None — сгенерируется 'CYL_i'
        """
        def_name = name if name else self._next_def("CYL")
        rot_str = self._to_rotation(rotation)

        geometry = f"Cylinder {{ radius {radius} height {height} }}"
        bounding = f"Cylinder {{ radius {radius} height {height} }}"

        return self._spawn_solid(
            def_name=def_name,
            translation=translation,
            rotation=rot_str,
            color=color,
            transparency=transparency,
            geometry_block=geometry,
            bounding_block=bounding,
            static=static,
            mass=mass,
        )

    def remove(self, def_name: str) -> bool:
        node = self.supervisor.getFromDef(def_name)
        if node is None:
            return False
        node.remove()
        self.supervisor.step(self.timestep)
        return True
