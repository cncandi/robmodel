<?php
/**
 * OnShape Proxy für RobModel
 * GET ?action=parts&url=...&ak=...&sk=...   → Teileliste (JSON)
 * GET ?action=stl&url=...&partId=...&ak=...&sk=...  → STL binary
 * GET ?action=type&url=...&ak=...&sk=...    → Element-Typ erkennen
 */

header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: GET, OPTIONS');
if ($_SERVER['REQUEST_METHOD'] === 'OPTIONS') exit;

$action  = $_GET['action']  ?? 'parts';
$url     = $_GET['url']     ?? '';
$ak      = trim($_GET['ak'] ?? '');
$sk      = trim($_GET['sk'] ?? '');
$partId  = $_GET['partId']  ?? '';
$units   = $_GET['units']   ?? 'millimeter';

if (!$url) { json_err('Keine URL'); }

// ── URL parsen ──────────────────────────────────────────────────
// https://cad.onshape.com/documents/{did}/w/{wid}/e/{eid}
// https://cad.onshape.com/documents/{did}/m/{mid}/e/{eid}
if (!preg_match('#/documents/([a-fA-F0-9]+)/([wm])/([a-fA-F0-9]+)/e/([a-fA-F0-9]+)#', $url, $m)) {
    json_err('Ungültige OnShape-URL. Format: .../documents/{did}/w/{wid}/e/{eid}');
}
[, $did, $wm, $wmid, $eid] = $m;

// ── Aktionen ───────────────────────────────────────────────────
switch ($action) {

case 'type':
    // Element-Typ aus dem Elements-Endpoint bestimmen
    $api = "https://cad.onshape.com/api/v6/documents/d/{$did}/{$wm}/{$wmid}/elements?elementId={$eid}";
    $res = os_get($api, $ak, $sk);
    if (!$res) json_err('Konnte Element-Typ nicht abrufen', 401);
    $data = json_decode($res, true);
    $el = is_array($data) ? ($data[0] ?? null) : null;
    header('Content-Type: application/json');
    echo json_encode([
        'type' => $el['elementType'] ?? 'unknown',
        'name' => $el['name']        ?? '',
    ]);
    break;

case 'parts':
    // Part Studio: Teile auflisten
    $api = "https://cad.onshape.com/api/v6/partstudios/d/{$did}/{$wm}/{$wmid}/e/{$eid}/parts?includeNonSolids=false";
    $res = os_get($api, $ak, $sk);
    if ($res === false) {
        // Fallback: Assembly-Instances
        $api2 = "https://cad.onshape.com/api/v6/assemblies/d/{$did}/{$wm}/{$wmid}/e/{$eid}";
        $res2 = os_get($api2, $ak, $sk);
        if ($res2 === false) json_err('API-Anfrage fehlgeschlagen – API-Keys prüfen', 401);
        $asm = json_decode($res2, true);
        $parts = [];
        foreach ($asm['rootAssembly']['instances'] ?? [] as $inst) {
            if (($inst['type'] ?? '') === 'Part') {
                $parts[] = [
                    'partId'     => $inst['partId']      ?? $inst['instanceId'],
                    'name'       => $inst['name']        ?? 'Part',
                    'bodyType'   => 'solid',
                    'elementId'  => $inst['elementId']   ?? $eid,
                    'documentId' => $inst['documentId']  ?? $did,
                    'wvmType'    => 'v',
                    'wvmId'      => $inst['documentMicroversionId'] ?? $wmid,
                    'source'     => 'assembly',
                ];
            }
        }
        header('Content-Type: application/json');
        echo json_encode($parts);
        break;
    }
    $parts_raw = json_decode($res, true) ?? [];
    $parts = array_values(array_filter($parts_raw, fn($p) => ($p['bodyType'] ?? '') === 'solid'));
    header('Content-Type: application/json');
    echo json_encode(array_map(fn($p) => [
        'partId'     => $p['partId'],
        'name'       => $p['name']       ?? 'Part',
        'bodyType'   => $p['bodyType'],
        'elementId'  => $eid,
        'documentId' => $did,
        'wvmType'    => $wm,
        'wvmId'      => $wmid,
        'source'     => 'partstudio',
    ], $parts));
    break;

case 'stl':
    if (!$partId) json_err('Keine partId');
    // Teile-Dokument kann sich von Haupt-Dokument unterscheiden (Assembly-Fall)
    $p_did  = $_GET['pdid']   ?? $did;
    $p_wm   = $_GET['pwm']    ?? $wm;
    $p_wmid = $_GET['pwmid']  ?? $wmid;
    $p_eid  = $_GET['peid']   ?? $eid;

    $api = "https://cad.onshape.com/api/v6/parts/d/{$p_did}/{$p_wm}/{$p_wmid}/e/{$p_eid}/partid/{$partId}/stl"
         . "?units={$units}&mode=binary&grouping=false";
    $res = os_get($api, $ak, $sk, true);
    if ($res === false) json_err('STL-Export fehlgeschlagen');
    header('Content-Type: application/octet-stream');
    header('Content-Length: ' . strlen($res));
    echo $res;
    break;

default:
    json_err('Unbekannte Aktion');
}

// ── Hilfsfunktionen ────────────────────────────────────────────
function os_get(string $url, string $ak, string $sk, bool $binary = false): string|false {
    $accept = $binary ? 'application/octet-stream' : 'application/json';
    $headers = ["Accept: {$accept}"];
    if ($ak && $sk) {
        $headers = array_merge($headers, os_auth($url, 'GET', '', $accept, $ak, $sk));
    }
    $ch = curl_init($url);
    curl_setopt_array($ch, [
        CURLOPT_RETURNTRANSFER => true,
        CURLOPT_FOLLOWLOCATION => true,
        CURLOPT_TIMEOUT        => 30,
        CURLOPT_HTTPHEADER     => $headers,
    ]);
    $body   = curl_exec($ch);
    $status = curl_getinfo($ch, CURLINFO_HTTP_CODE);
    curl_close($ch);
    return ($status >= 200 && $status < 300 && $body !== false) ? $body : false;
}

function os_auth(string $url, string $method, string $ct, string $accept, string $ak, string $sk): array {
    $parsed = parse_url($url);
    $path   = strtolower(($parsed['path'] ?? '/') . (isset($parsed['query']) ? '?' . $parsed['query'] : ''));
    $date   = gmdate('D, d M Y H:i:s') . ' GMT';
    $nonce  = strtoupper(bin2hex(random_bytes(12)));
    $str    = implode("\n", [
        strtolower($method), strtolower($ct), strtolower($accept),
        strtolower($date), strtolower($nonce), $path,
    ]);
    $sig = base64_encode(hash_hmac('sha256', $str, $sk, true));
    return [
        "Date: {$date}",
        "On-Nonce: {$nonce}",
        "Authorization: On {$ak}:HmacSHA256:{$sig}",
    ];
}

function json_err(string $msg, int $code = 400): never {
    http_response_code($code);
    header('Content-Type: application/json');
    echo json_encode(['error' => $msg]);
    exit;
}
