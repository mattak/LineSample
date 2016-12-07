using System.Linq;
using UnityEngine;

public static class LinePolygon
{
    public static Mesh CreateMesh(Vector2[] points, float width)
    {
        Vector2[] verts = CreateLineVerticies(points, width);
        int[] triangles = CreateTriangles(points.Length);

        Mesh mesh = new Mesh();
        mesh.vertices = ConvertVector2ToVector3(verts);
        mesh.triangles = triangles;
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
        return mesh;
    }

    public static Vector3[] ConvertVector2ToVector3(Vector2[] points)
    {
        return points.Select(it => new Vector3(it.x, 0, it.y)).ToArray();
    }

    public static Vector2[] CreateLineVerticies(Vector2[] points, float width)
    {
        if (points.Length < 2)
        {
            return null;
        }

        int verts_length = points.Length << 1;
        Vector2[] result = new Vector2[verts_length];
        Vector2[] p1_updown, p2_updown;
        float theta, sin_theta, cos_theta;
        float theta_previous, sin_theta_previous, cos_theta_previous;

        // 始点を求める
        {
            theta_previous = Mathf.Atan2(points[1].y - points[0].y, points[1].x - points[0].x);
            sin_theta_previous = Mathf.Sin(theta_previous);
            cos_theta_previous = Mathf.Cos(theta_previous);

            p1_updown = CalculateUpDownPoints(points[0], sin_theta_previous, cos_theta_previous, width);
            result[0] = p1_updown[0];
            result[1] = p1_updown[1];
        }

        // 中点を求める
        for (int i = 1; i < points.Length - 1; i++)
        {
            theta = Mathf.Atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
            sin_theta = Mathf.Sin(theta);
            cos_theta = Mathf.Cos(theta);

            p2_updown = CalculateUpDownPoints(points[i], sin_theta, cos_theta, width);

            Vector2[] medium_points = CalculateMidiumPoint(
                p1_updown,
                p2_updown,
                sin_theta_previous,
                cos_theta_previous,
                sin_theta,
                cos_theta);

            int i2 = i << 1; // i*2
            result[i2] = medium_points[0];
            result[i2 + 1] = medium_points[1];

            sin_theta_previous = sin_theta;
            cos_theta_previous = cos_theta;
            p1_updown = p2_updown;
        }

        // 終点を求める
        {
            int length = points.Length;
            theta = Mathf.Atan2(points[length - 1].y - points[length - 2].y, points[length - 1].x - points[length - 2].x);
            sin_theta = Mathf.Sin(theta);
            cos_theta = Mathf.Cos(theta);

            p2_updown = CalculateUpDownPoints(points[length - 1], sin_theta, cos_theta, width);
            result[verts_length - 2] = p2_updown[0];
            result[verts_length - 1] = p2_updown[1];
        }

        return result;
    }

    public static int[] CreateTriangles(int point_count)
    {
        int[] triangles = new int[(point_count - 1) * 6];

        for (int i = 0; i < point_count - 1; i++)
        {
            int triangle_index = i * 6;
            int vert_index = i << 1;
            triangles[triangle_index] = vert_index;
            triangles[triangle_index + 1] = vert_index + 1;
            triangles[triangle_index + 2] = vert_index + 2;
            triangles[triangle_index + 3] = vert_index + 2;
            triangles[triangle_index + 4] = vert_index + 1;
            triangles[triangle_index + 5] = vert_index + 3;
        }

        return triangles;
    }

    public static Vector2[] CalculateUpDownPoints(Vector2 p, float sin_theta, float cos_theta, float width)
    {
        float width_cos_theta = width * cos_theta;
        float width_sin_theta = width * sin_theta;
        Vector2 p_up = new Vector2(p.x + width_sin_theta, p.y - width_cos_theta);
        Vector2 p_down = new Vector2(p.x - width_sin_theta, p.y + width_cos_theta);
        return new Vector2[] {p_up, p_down};
    }

    // Input: p1, p2, p12_theta, p23_theta
    // Output: p2', p2''
    public static Vector2[] CalculateMidiumPoint(
        Vector2[] p1_updown,
        Vector2[] p2_updown,
        float sin_theta1, float cos_theta1, float sin_theta2, float cos_theta2)
    {
        float a1b2_minus_a2b1 = -sin_theta1 * cos_theta2 + sin_theta2 * cos_theta1;

        if (a1b2_minus_a2b1 == 0f)
        {
            return new Vector2[]
            {
                (p1_updown[0] + p2_updown[0]) * 0.5f,
                (p1_updown[1] + p2_updown[1]) * 0.5f,
            };
        }

        Vector2 p1_up = p1_updown[0];
        Vector2 p1_down = p1_updown[1];
        Vector2 p2_up = p2_updown[0];
        Vector2 p2_down = p2_updown[1];

        float c1b2_c2b1_up = (p1_up.x * sin_theta1 - p1_up.y * cos_theta1) * cos_theta2 -
                             (p2_up.x * sin_theta2 - p2_up.y * cos_theta2) * cos_theta1;

        float c1a2_c2a1_up = (p1_up.x * sin_theta1 - p1_up.y * cos_theta1) * -sin_theta2 +
                             (p2_up.x * sin_theta2 - p2_up.y * cos_theta2) * sin_theta1;

        float c1b2_c2b1_down = (p1_down.x * sin_theta1 - p1_down.y * cos_theta1) * cos_theta2 -
                               (p2_down.x * sin_theta2 - p2_down.y * cos_theta2) * cos_theta1;
        float c1a2_c2a1_down = (p1_down.x * sin_theta1 - p1_down.y * cos_theta1) * -sin_theta2 +
                               (p2_down.x * sin_theta2 - p2_down.y * cos_theta2) * sin_theta1;

        return new Vector2[]
        {
            new Vector2(c1b2_c2b1_up / -a1b2_minus_a2b1, c1a2_c2a1_up / a1b2_minus_a2b1),
            new Vector2(c1b2_c2b1_down / -a1b2_minus_a2b1, c1a2_c2a1_down / a1b2_minus_a2b1),
        };
    }
}