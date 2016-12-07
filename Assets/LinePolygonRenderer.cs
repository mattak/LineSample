using UnityEngine;

public class LinePolygonRenderer : MonoBehaviour
{
    public Material Mateiral;

    void Start()
    {
        CreateLine(new[]
        {
            new Vector2(-8, 0),
            new Vector2(0, -5),
            new Vector2(8, 0),
        });

        CreateLine(new[]
        {
            new Vector2(-10, 5),
            new Vector2(-10, 15),
        });

        CreateLine(new[]
        {
            new Vector2(10, 5),
            new Vector2(10, 15),
        });

        CreateLine(new[]
        {
            new Vector2(20, -5),
            new Vector2(25, 5),
            new Vector2(20, 15),
        });

        CreateLine(new[]
        {
            new Vector2(-20, -5),
            new Vector2(-25, 5),
            new Vector2(-20, 15),
        });
    }

    void CreateLine(Vector2[] points)
    {
        var _object = new GameObject("PolygonLine");
        var meshFilter = _object.AddComponent<MeshFilter>();
        var meshRenderer = _object.AddComponent<MeshRenderer>();

        meshFilter.sharedMesh = LinePolygon.CreateMesh(points, 1f);
        meshRenderer.material = this.Mateiral;
    }
}