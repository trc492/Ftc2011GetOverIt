var NAVTREE =
[
  [ "TRC Library for FTC (2011)", "index.html", [
    [ "Data Structures", "annotated.html", [
      [ "ACCEL", "struct_a_c_c_e_l.html", null ],
      [ "BATT", "struct_b_a_t_t.html", null ],
      [ "DRIVE", "struct_d_r_i_v_e.html", null ],
      [ "GYRO", "struct_g_y_r_o.html", null ],
      [ "JOYBTN", "struct_j_o_y_b_t_n.html", null ],
      [ "LNFOLLOW", "struct_l_n_f_o_l_l_o_w.html", null ],
      [ "MENU", "struct_m_e_n_u.html", null ],
      [ "NXTBTN", "struct_n_x_t_b_t_n.html", null ],
      [ "PIDCTRL", "struct_p_i_d_c_t_r_l.html", null ],
      [ "PIDDRIVE", "struct_p_i_d_d_r_i_v_e.html", null ],
      [ "ROBOTTASK", "struct_r_o_b_o_t_t_a_s_k.html", null ],
      [ "SENSOR", "struct_s_e_n_s_o_r.html", null ],
      [ "SERVO", "struct_s_e_r_v_o.html", null ],
      [ "SM", "struct_s_m.html", null ],
      [ "TASKS", "struct_t_a_s_k_s.html", null ],
      [ "TIMER", "struct_t_i_m_e_r.html", null ],
      [ "TOUCH", "struct_t_o_u_c_h.html", null ],
      [ "WAIT_EVT", "struct_w_a_i_t___e_v_t.html", null ]
    ] ],
    [ "Data Structure Index", "classes.html", null ],
    [ "Data Fields", "functions.html", null ],
    [ "File List", "files.html", [
      [ "C:/Users/Michael/Ftc/2011/code/trclib/accel.h", "accel_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/batt.h", "batt_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/dbgtrace.h", "dbgtrace_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/drive.h", "drive_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/gyro.h", "gyro_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/joybtn.h", "joybtn_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/lnfollow.h", "lnfollow_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/menu.h", "menu_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/nxtbtn.h", "nxtbtn_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/pidctrl.h", "pidctrl_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/piddrive.h", "piddrive_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/sensor.h", "sensor_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/servo.h", "servo_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/sm.h", "sm_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/task.h", "task_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/timer.h", "timer_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/touch.h", "touch_8h.html", null ],
      [ "C:/Users/Michael/Ftc/2011/code/trclib/trcdefs.h", "trcdefs_8h.html", null ]
    ] ],
    [ "Globals", "globals.html", null ]
  ] ]
];

function createIndent(o,domNode,node,level)
{
  if (node.parentNode && node.parentNode.parentNode)
  {
    createIndent(o,domNode,node.parentNode,level+1);
  }
  var imgNode = document.createElement("img");
  if (level==0 && node.childrenData)
  {
    node.plus_img = imgNode;
    node.expandToggle = document.createElement("a");
    node.expandToggle.href = "javascript:void(0)";
    node.expandToggle.onclick = function() 
    {
      if (node.expanded) 
      {
        $(node.getChildrenUL()).slideUp("fast");
        if (node.isLast)
        {
          node.plus_img.src = node.relpath+"ftv2plastnode.png";
        }
        else
        {
          node.plus_img.src = node.relpath+"ftv2pnode.png";
        }
        node.expanded = false;
      } 
      else 
      {
        expandNode(o, node, false);
      }
    }
    node.expandToggle.appendChild(imgNode);
    domNode.appendChild(node.expandToggle);
  }
  else
  {
    domNode.appendChild(imgNode);
  }
  if (level==0)
  {
    if (node.isLast)
    {
      if (node.childrenData)
      {
        imgNode.src = node.relpath+"ftv2plastnode.png";
      }
      else
      {
        imgNode.src = node.relpath+"ftv2lastnode.png";
        domNode.appendChild(imgNode);
      }
    }
    else
    {
      if (node.childrenData)
      {
        imgNode.src = node.relpath+"ftv2pnode.png";
      }
      else
      {
        imgNode.src = node.relpath+"ftv2node.png";
        domNode.appendChild(imgNode);
      }
    }
  }
  else
  {
    if (node.isLast)
    {
      imgNode.src = node.relpath+"ftv2blank.png";
    }
    else
    {
      imgNode.src = node.relpath+"ftv2vertline.png";
    }
  }
  imgNode.border = "0";
}

function newNode(o, po, text, link, childrenData, lastNode)
{
  var node = new Object();
  node.children = Array();
  node.childrenData = childrenData;
  node.depth = po.depth + 1;
  node.relpath = po.relpath;
  node.isLast = lastNode;

  node.li = document.createElement("li");
  po.getChildrenUL().appendChild(node.li);
  node.parentNode = po;

  node.itemDiv = document.createElement("div");
  node.itemDiv.className = "item";

  node.labelSpan = document.createElement("span");
  node.labelSpan.className = "label";

  createIndent(o,node.itemDiv,node,0);
  node.itemDiv.appendChild(node.labelSpan);
  node.li.appendChild(node.itemDiv);

  var a = document.createElement("a");
  node.labelSpan.appendChild(a);
  node.label = document.createTextNode(text);
  a.appendChild(node.label);
  if (link) 
  {
    a.href = node.relpath+link;
  } 
  else 
  {
    if (childrenData != null) 
    {
      a.className = "nolink";
      a.href = "javascript:void(0)";
      a.onclick = node.expandToggle.onclick;
      node.expanded = false;
    }
  }

  node.childrenUL = null;
  node.getChildrenUL = function() 
  {
    if (!node.childrenUL) 
    {
      node.childrenUL = document.createElement("ul");
      node.childrenUL.className = "children_ul";
      node.childrenUL.style.display = "none";
      node.li.appendChild(node.childrenUL);
    }
    return node.childrenUL;
  };

  return node;
}

function showRoot()
{
  var headerHeight = $("#top").height();
  var footerHeight = $("#nav-path").height();
  var windowHeight = $(window).height() - headerHeight - footerHeight;
  navtree.scrollTo('#selected',0,{offset:-windowHeight/2});
}

function expandNode(o, node, imm)
{
  if (node.childrenData && !node.expanded) 
  {
    if (!node.childrenVisited) 
    {
      getNode(o, node);
    }
    if (imm)
    {
      $(node.getChildrenUL()).show();
    } 
    else 
    {
      $(node.getChildrenUL()).slideDown("fast",showRoot);
    }
    if (node.isLast)
    {
      node.plus_img.src = node.relpath+"ftv2mlastnode.png";
    }
    else
    {
      node.plus_img.src = node.relpath+"ftv2mnode.png";
    }
    node.expanded = true;
  }
}

function getNode(o, po)
{
  po.childrenVisited = true;
  var l = po.childrenData.length-1;
  for (var i in po.childrenData) 
  {
    var nodeData = po.childrenData[i];
    po.children[i] = newNode(o, po, nodeData[0], nodeData[1], nodeData[2],
        i==l);
  }
}

function findNavTreePage(url, data)
{
  var nodes = data;
  var result = null;
  for (var i in nodes) 
  {
    var d = nodes[i];
    if (d[1] == url) 
    {
      return new Array(i);
    }
    else if (d[2] != null) // array of children
    {
      result = findNavTreePage(url, d[2]);
      if (result != null) 
      {
        return (new Array(i).concat(result));
      }
    }
  }
  return null;
}

function initNavTree(toroot,relpath)
{
  var o = new Object();
  o.toroot = toroot;
  o.node = new Object();
  o.node.li = document.getElementById("nav-tree-contents");
  o.node.childrenData = NAVTREE;
  o.node.children = new Array();
  o.node.childrenUL = document.createElement("ul");
  o.node.getChildrenUL = function() { return o.node.childrenUL; };
  o.node.li.appendChild(o.node.childrenUL);
  o.node.depth = 0;
  o.node.relpath = relpath;

  getNode(o, o.node);

  o.breadcrumbs = findNavTreePage(toroot, NAVTREE);
  if (o.breadcrumbs == null)
  {
    o.breadcrumbs = findNavTreePage("index.html",NAVTREE);
  }
  if (o.breadcrumbs != null && o.breadcrumbs.length>0)
  {
    var p = o.node;
    for (var i in o.breadcrumbs) 
    {
      var j = o.breadcrumbs[i];
      p = p.children[j];
      expandNode(o,p,true);
    }
    p.itemDiv.className = p.itemDiv.className + " selected";
    p.itemDiv.id = "selected";
    $(window).load(showRoot);
  }
}

