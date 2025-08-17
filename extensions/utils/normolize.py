import re

def normalize_objects(objs):
    if objs is None:
        return []
    if len(objs) == 1 and ("," in objs[0]):
        return [s for s in objs[0].split(",") if s]
    return objs

def split_floats(s: str):
    # разбиваем по , ; и пробелам
    return [float(x) for x in re.split(r"[,\s;]+", s.strip()) if x]

def chunk6(vals):
    if len(vals) % 6 != 0:
        raise ValueError(f"Количество чисел в --target-poses должно быть кратно 6, получено {len(vals)}.")
    return [vals[i:i+6] for i in range(0, len(vals), 6)]

def normalize_target_poses(tp_tokens):
    """
    Возвращает список поз: [[x,y,z,a,b,c], ...]
    Поддерживает:
      --target-poses X Y Z A B C            (одна поза)
      --target-poses X...C X2...C2          (несколько поз, кратно 6)
      --target-poses "X Y Z A B C; X2 Y2 Z2 A2 B2 C2"
      --target-poses "X,Y,Z,A,B,C;X2,Y2,Z2,A2,B2,C2"
    """
    if tp_tokens is None or len(tp_tokens) == 0:
        raise ValueError("Не заданы --target-poses")

    if len(tp_tokens) == 1:
        vals = split_floats(tp_tokens[0])
        return chunk6(vals) 
    try:
        vals = [float(x) for x in tp_tokens]
    except ValueError:
        vals = split_floats(" ".join(tp_tokens))
    return chunk6(vals)

def normalize_vec(tokens, expect_len):
    """Принимает ['1','2','3'] ИЛИ ['1,2,3'] ИЛИ ['1 2 3'] и т.п."""
    if tokens is None:
        return None
    if len(tokens) == expect_len:
        return [float(x) for x in tokens]
    if len(tokens) == 1:
        parts = [p for p in re.split(r"[,\s;]+", tokens[0].strip()) if p]
        if len(parts) == expect_len:
            return [float(x) for x in parts]
    raise ValueError(f"Ожидал {expect_len} значений, получил: {tokens}")


def have_pallet(opts):
    """Палета считается заданной, если есть origin, nx, ny, sx, sy (nz/sz/рор — опционально)."""
    return (
        opts.pallet_origin is not None
        and opts.pallet_nx is not None
        and opts.pallet_ny is not None
        and (opts.pallet_sx is not None)
        and (opts.pallet_sy is not None)
    )