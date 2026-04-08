// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/completion.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/powercap.h>
#include <linux/slab.h>

#define SPEL_HW "spel-hw"

#define MMIO_OFFSET(base, offset) ((void __iomem *)((uintptr_t)(base) + (uintptr_t)(offset)))

#define SUSPICIOUS_VALUE   0xFFFFFFFF
#define SPEL_NAME_MAX      32
#define ENERGY_UNITS_MASK  0xF0000
#define TIME_UNITS_MASK    0xF00
#define POWER_UNITS_MASK   0x7

#define ENERGY_UNITS_SHIFT 16
#define TIME_UNITS_SHIFT   8
#define POWER_UNITS_SHIFT  0

#define MAX_POWER_LIMIT    0x00007FFF
#define ENABLE_POWER_LIMIT 0x80000000

#define MAX_TIME_WINDOW    0x003FFFFF
#define MIN_TIME_WINDOW_IN_MS 10
#define TIME_WINDOW_MASK_L 0x00007FFF
#define TIME_WINDOW_MASK_H 0x003F8000

#define SYS_NAME "sys"
#define SOC_NAME "soc"
#define SOC_CL0_NAME "cl0"
#define SOC_CL1_NAME "cl1"
#define SOC_CL2_NAME "cl2"
#define SOC_IGPU_NAME "igpu"
#define SOC_DGPU_NAME "dgpu"
#define SOC_NSP_NAME "nsp"
#define SOC_MMCX_NAME "mmcx"
#define SOC_INFRA_NAME "infra"
#define SOC_DRAM_NAME "dram"
#define SOC_MDM_NAME "mdm"
#define SOC_WLAN_NAME "wlan"

#define SOC_USB1_NAME "usb1"
#define SOC_USB2_NAME "usb2"
#define SOC_USB3_NAME "usb3"

#define CONSTRAINT_NAME_SYS_PL1 "sys pl1"
#define CONSTRAINT_NAME_SYS_PL2 "sys pl2"
#define CONSTRAINT_NAME_SYS_PL3 "sys pl3"
#define CONSTRAINT_NAME_SYS_PL4 "sys pl4"
#define CONSTRAINT_NAME_SOC_PL1 "soc pl1"
#define CONSTRAINT_NAME_SOC_PL2 "soc pl2"
#define CONSTRAINT_NAME_SOC_PL3 "soc pl3"
#define CONSTRAINT_NAME_SOC_PL4 "soc pl4"
#define CONSTRAINT_NAME_UNDEFINED "unknown"

#define ROOT_NODE "root"

#define REPORT_UNITS_OFFSET 4
#define LIMITS_CAPABILITY_OFFSET 32

enum CHILD_NODE {
	SYS,
	SOC,
	SOC_CL0,
	SOC_CL1,
	SOC_CL2,
	SOC_IGPU,
	SOC_DGPU,
	SOC_NSP,
	SOC_MMCX,
	SOC_INFRA,
	SOC_DRAM,
	SOC_MDM,
	SOC_WLAN,
	SOC_USB1,
	SOC_USB2,
	SOC_USB3,
};

struct spel_ops;
struct spel_priv;

struct spel_units {
	u32 energy;
	u32 time;
	u32 power;
};

struct spel_constraints {
	struct list_head pzc_list;
	struct powercap_zone_constraint *pzc;
};

struct spel {
	struct powercap_zone zone;
	struct list_head sibling;
	struct list_head children;
	struct spel_ops *ops;
	char name[SPEL_NAME_MAX];
	struct device_node *np;
	int count;
	bool enabled;
	void __iomem *address;
	int nr_constraints;
	struct list_head pz_list;
	struct list_head node;
	struct mutex lock;
	struct spel *parent;
	struct spel_constraints *constraints_list;
	struct spel_priv *priv;
};

struct spel_ops {
	int (*set_enable)(struct spel *spel, bool enable);
	int (*get_enable)(struct spel *spel, bool *enable);
	u64 (*get_spel_energy_uj)(struct spel *spel);
	void (*release)(struct spel *spel);
};

struct spel_node {
	char name[SPEL_NAME_MAX];
	struct spel_node *parent;
	struct spel *spel;
	struct device_node *np;
	void __iomem *address;
	int nr_constraints;
};

struct child_spec {
	enum CHILD_NODE	index;
	char		*name;
	uintptr_t	offset;
	int		nr_constraints;
};

struct spel_hierarchy {
	int count;
	int cur_index;
	struct powercap_control_type *pct;
	struct spel_node *hierarchy;
	struct spel *root;
};

/* Driver private data structure - replaces all static globals */
struct spel_priv {
	struct device *dev;
	void __iomem *nodes;
	void __iomem *constraints;
	void __iomem *config;
	struct spel_units budget_units;
	struct spel_units report_units;
	struct list_head spel_list;
	struct mutex spel_list_lock;
	struct spel_hierarchy *sh;
};

struct constraint_spec {
	int		cid;
	char		*zone_name;
	char		*constraint_name;
	uintptr_t	limit_offset;
	uintptr_t	time_window_offset;
	u32		supported_mask;
	bool		supported;
};

/* Global pointer to access priv from control type callbacks */
static struct spel_priv *g_spel_priv;

static struct child_spec children_nodes[] = {
	{ SYS,       SYS_NAME,       0x40, 4 },
	{ SOC,       SOC_NAME,       0x00, 4 },
	{ SOC_CL0,   SOC_CL0_NAME,   0x5C, 0 },
	{ SOC_CL1,   SOC_CL1_NAME,   0x60, 0 },
	{ SOC_CL2,   SOC_CL2_NAME,   0x64, 0 },
	{ SOC_IGPU,  SOC_IGPU_NAME,  0x08, 0 },
	{ SOC_DGPU,  SOC_DGPU_NAME,  0x44, 0 },
	{ SOC_NSP,   SOC_NSP_NAME,   0x0C, 0 },
	{ SOC_MMCX,  SOC_MMCX_NAME,  0x10, 0 },
	{ SOC_INFRA, SOC_INFRA_NAME, 0x18, 0 },
	{ SOC_DRAM,  SOC_DRAM_NAME,  0x1C, 0 },
	{ SOC_MDM,   SOC_MDM_NAME,   0x48, 0 },
	{ SOC_WLAN,  SOC_WLAN_NAME,  0x4C, 0 },
	{ SOC_USB1,  SOC_USB1_NAME,  0x50, 0 },
	{ SOC_USB2,  SOC_USB2_NAME,  0x54, 0 },
	{ SOC_USB3,  SOC_USB3_NAME,  0x58, 0 },
};

static struct constraint_spec constraints[] = {
	{ -1, SYS_NAME, CONSTRAINT_NAME_SYS_PL1, 0x10, 0x70, 0x01, false },
	{ -1, SYS_NAME, CONSTRAINT_NAME_SYS_PL2, 0x14, 0x74, 0x02, false },
	{ -1, SYS_NAME, CONSTRAINT_NAME_SYS_PL3, 0x18, 0x78, 0x04, false },
	{ -1, SYS_NAME, CONSTRAINT_NAME_SYS_PL4, 0x1C, 0x7C, 0x08, false },
	{ -1, SOC_NAME, CONSTRAINT_NAME_SOC_PL1, 0x00, 0x60, 0x10, false },
	{ -1, SOC_NAME, CONSTRAINT_NAME_SOC_PL2, 0x04, 0x64, 0x20, false },
	{ -1, SOC_NAME, CONSTRAINT_NAME_SOC_PL3, 0x08, 0x68, 0x40, false },
	{ -1, SOC_NAME, CONSTRAINT_NAME_SOC_PL4, 0x0C, 0x6C, 0x80, false },
};

static inline struct spel *to_spel(struct powercap_zone *zone)
{
	return container_of(zone, struct spel, zone);
}

static int get_spel_count(void)
{
	return ARRAY_SIZE(children_nodes);
}

static int get_constraint_count(void)
{
	return ARRAY_SIZE(constraints);
}

static u32 spel_readl(void __iomem *addr)
{
	u32 val;

	val = readl(addr);
	if (val == SUSPICIOUS_VALUE)
		pr_warn("Suspicious register read value 0x%X\n", SUSPICIOUS_VALUE);

	return val;
}

static void spel_writel(u32 val, void __iomem *addr)
{
	pr_debug("writing %d to address: %p\n", val, addr);

	writel(val, addr);
}

static int spel_get_units(struct spel_priv *priv, void __iomem *address)
{
	u32 val;

	val = spel_readl(address);

	priv->budget_units.energy = (val & ENERGY_UNITS_MASK) >> ENERGY_UNITS_SHIFT;
	priv->budget_units.time = (val & TIME_UNITS_MASK) >> TIME_UNITS_SHIFT;
	priv->budget_units.power = (val & POWER_UNITS_MASK) >> POWER_UNITS_SHIFT;

	val = spel_readl(MMIO_OFFSET(address, REPORT_UNITS_OFFSET));

	priv->report_units.energy = (val & ENERGY_UNITS_MASK) >> ENERGY_UNITS_SHIFT;
	priv->report_units.time = (val & TIME_UNITS_MASK) >> TIME_UNITS_SHIFT;
	priv->report_units.power = (val & POWER_UNITS_MASK) >> POWER_UNITS_SHIFT;

	dev_dbg(priv->dev, "budget_units.energy: %d\n", priv->budget_units.energy);
	dev_dbg(priv->dev, "budget_units.time: %d\n", priv->budget_units.time);
	dev_dbg(priv->dev, "budget_units.power: %d\n", priv->budget_units.power);
	dev_dbg(priv->dev, "report_units.energy: %d\n", priv->report_units.energy);
	dev_dbg(priv->dev, "report_units.time: %d\n", priv->report_units.time);
	dev_dbg(priv->dev, "report_units.power: %d\n", priv->report_units.power);

	return 0;
}

static bool constraint_is_supported(void __iomem *address, u32 bit_mask)
{
	u32 val;

	val = spel_readl(MMIO_OFFSET(address, LIMITS_CAPABILITY_OFFSET)) & bit_mask;

	return val != 0;
}

static void update_constraints(struct child_spec *node)
{
	int cid = 0;

	for (int i = 0; i < get_constraint_count(); ++i)
		if (!strcmp(node->name, constraints[i].zone_name) &&
				constraints[i].supported) {
			constraints[i].cid = cid;
			cid++;
		}
	node->nr_constraints = cid;
}

static void spel_init_constraints(void __iomem *address)
{
	for (int i = 0; i < get_constraint_count(); ++i)
		if (constraint_is_supported(address, constraints[i].supported_mask))
			constraints[i].supported = true;

	for (int i = 0; i < get_spel_count(); ++i)
		if (children_nodes[i].nr_constraints)
			update_constraints(&children_nodes[i]);
}

static int hw_set_enable(struct spel *spel, bool enable)
{
	return 0;
}

static int hw_get_enable(struct spel *spel, bool *enable)
{
	*enable = true;

	return 0;
}

static u64 hw_get_spel_energy_uj(struct spel *spel)
{
	u64 energy = spel_readl(spel->address);

	return (energy * 1000000) >> spel->priv->report_units.energy;
}

static void hw_release(struct spel *spel)
{
}

static int decode_time_window(struct spel_priv *priv, u32 *time_window)
{
	u32 window = *time_window;
	u32 final_window = 0;

	final_window = (window & (TIME_WINDOW_MASK_H << 1)) >> 1;
	final_window |= window & TIME_WINDOW_MASK_L;
	final_window = (final_window * 1000UL) >> priv->budget_units.time;
	*time_window = final_window;

	return 0;
}

static int get_time_window_us(struct powercap_zone *pcz, int cid, u64 *time_window)
{
	struct spel *spel = to_spel(pcz);
	struct spel_priv *priv = spel->priv;
	u32 window;
	int ret;

	for (int i = 0; i < get_constraint_count(); i++)
		if (!strcmp(pcz->name, constraints[i].zone_name) &&
				constraints[i].cid == cid) {
			window = (u32)spel_readl(MMIO_OFFSET(
				priv->constraints,
				constraints[i].time_window_offset));
			ret = decode_time_window(priv, &window);
			if (ret)
				return ret;
			*time_window = (u64)window;
			return 0;
		}

	return -EINVAL;
}

static int encode_time_window(struct spel_priv *priv, u32 *time_window)
{
	u32 window;
	u32 final_window = 0;


	window = mul_u64_u32_div(*time_window, (1U << priv->budget_units.time),
					1000UL);

	if (window > MAX_TIME_WINDOW || window < MIN_TIME_WINDOW_IN_MS) {
		dev_err(priv->dev, "time window out of range value: %d\n", window);
		return -EINVAL;
	}
	final_window = (window & TIME_WINDOW_MASK_H) << 1;
	final_window |= window & TIME_WINDOW_MASK_L;
	*time_window = final_window;


	return 0;
}

static int set_time_window_us(struct powercap_zone *pcz, int cid, u64 time_window)
{
	struct spel *spel = to_spel(pcz);
	struct spel_priv *priv = spel->priv;
	u32 window = (u32)time_window;
	int ret;

	for (int i = 0; i < get_constraint_count(); i++) {
		if (!strcmp(pcz->name, constraints[i].zone_name) &&
				constraints[i].cid == cid) {
			ret = encode_time_window(priv, &window);
			if (ret)
				return ret;
			spel_writel(window,
				MMIO_OFFSET(priv->constraints,
				constraints[i].time_window_offset));
			return 0;
		}
	}

	return -EINVAL;
}

static int decode_power_limit(struct spel_priv *priv, u64 *power_limit)
{
	u64 limit = *power_limit & MAX_POWER_LIMIT;

	limit = limit * 1000000ULL >> priv->budget_units.power;
	*power_limit = limit;

	return 0;
}

static int get_power_limit_uw(struct powercap_zone *pcz,
			      int cid, u64 *power_limit)
{
	struct spel *spel = to_spel(pcz);
	struct spel_priv *priv = spel->priv;
	u64 limit;
	int ret;

	for (int i = 0; i < get_constraint_count(); i++) {
		if (!strcmp(pcz->name, constraints[i].zone_name) &&
				constraints[i].cid == cid) {
			limit = (u64)spel_readl(MMIO_OFFSET(
				priv->constraints,
				constraints[i].limit_offset));
			ret = decode_power_limit(priv, &limit);
			if (ret)
				return ret;
			*power_limit = limit;
			return 0;
		}
	}

	return -EINVAL;
}

static int encode_power_limit(struct spel_priv *priv, u64 *power_limit)
{
	u64 limit;

	limit = DIV_ROUND_UP_ULL(*power_limit << priv->budget_units.power,
					1000000ULL);

	if (limit > MAX_POWER_LIMIT) {
		dev_err(priv->dev, "power limit exceeds maximum value: %d\n",
			MAX_POWER_LIMIT);
		return -EINVAL;
	}
	limit |= ENABLE_POWER_LIMIT;
	*power_limit = limit;

	return 0;
}

static int set_power_limit_uw(struct powercap_zone *pcz,
			      int cid, u64 power_limit)
{
	struct spel *spel = to_spel(pcz);
	struct spel_priv *priv = spel->priv;
	int ret;

	for (int i = 0; i < get_constraint_count(); i++) {
		if (!strcmp(pcz->name, constraints[i].zone_name) &&
				constraints[i].cid == cid) {
			ret = encode_power_limit(priv, &power_limit);
			if (ret)
				return ret;
			spel_writel(power_limit,
				MMIO_OFFSET(priv->constraints,
				constraints[i].limit_offset));
			return 0;
		}
	}

	return -EINVAL;
}

static const char *get_constraint_name(struct powercap_zone *pcz, int cid)
{
	for (int i = 0; i < get_constraint_count(); i++)
		if (!strcmp(pcz->name, constraints[i].zone_name) &&
				constraints[i].cid == cid)
			return constraints[i].constraint_name;

	return CONSTRAINT_NAME_UNDEFINED;
}

static int get_max_power_uw(struct powercap_zone *pcz, int id, u64 *max_power)
{
	return -EOPNOTSUPP;
}

static int __get_energy_uj(struct spel *spel, u64 *energy_uj)
{
	*energy_uj = spel->ops->get_spel_energy_uj(spel);

	return 0;
}

static int get_energy_uj(struct powercap_zone *pcz, u64 *energy_uj)
{
	int ret = 0;
	struct spel *spel = to_spel(pcz);

	mutex_lock(&spel->lock);
	ret = __get_energy_uj(spel, energy_uj);
	mutex_unlock(&spel->lock);

	return ret;
}

static void spel_clean_up_constraints(struct spel_priv *priv, struct spel *spel)
{
	struct spel_constraints *constraint, *aux;

	if (!spel->constraints_list)
		return;

	constraint = spel->constraints_list;
	list_for_each_entry_safe(constraint, aux,
			&spel->constraints_list->pzc_list, pzc_list) {
		kfree(constraint->pzc);
		constraint->pzc = NULL;
		mutex_lock(&priv->spel_list_lock);
		list_del(&constraint->pzc_list);
		mutex_unlock(&priv->spel_list_lock);
		kfree(constraint);
	}

	/* Free the head node as well */
	kfree(spel->constraints_list->pzc);
	kfree(spel->constraints_list);
	spel->constraints_list = NULL;
}

static int spel_release_zone(struct powercap_zone *pcz)
{
	struct spel *spel = to_spel(pcz);
	struct spel *parent = spel->parent;
	struct spel_priv *priv = spel->priv;

	if (!list_empty(&spel->children))
		return -EBUSY;

	if (parent)
		list_del(&spel->sibling);

	if (spel->ops) {
		spel->ops->release(spel);
	} else {
		mutex_lock(&priv->spel_list_lock);
		list_del(&spel->node);
		mutex_unlock(&priv->spel_list_lock);
		spel_clean_up_constraints(priv, spel);
		kfree(spel);
	}

	return 0;
}

static struct powercap_zone_constraint_ops constraint_ops = {
	.set_power_limit_uw = set_power_limit_uw,
	.get_power_limit_uw = get_power_limit_uw,
	.set_time_window_us = set_time_window_us,
	.get_time_window_us = get_time_window_us,
	.get_max_power_uw = get_max_power_uw,
	.get_name = get_constraint_name,
};

static struct powercap_zone_ops zone_ops = {
	.get_energy_uj = get_energy_uj,
	.release = spel_release_zone,
};

static struct spel_ops hw_ops = {
	.set_enable = hw_set_enable,
	.get_enable = hw_get_enable,
	.get_spel_energy_uj = hw_get_spel_energy_uj,
	.release = hw_release,
};

static void spel_ops_init(struct spel *spel, struct spel_ops *ops)
{
	if (spel) {
		INIT_LIST_HEAD(&spel->children);
		INIT_LIST_HEAD(&spel->sibling);
		spel->ops = ops;
	}
}

static void spel_unregister(struct spel_priv *priv, struct spel *spel)
{
	if (!spel)
		return;

	dev_dbg(priv->dev, "Unregistered spel node '%s'\n", spel->zone.name);

	powercap_unregister_zone(priv->sh->pct, &spel->zone);
}

static int spel_add_constraint(struct spel_priv *priv, struct spel *spel, int constraint_id)
{
	struct spel_constraints *constraints_list;
	struct powercap_zone_constraint *pzc;

	constraints_list = devm_kzalloc(priv->dev, sizeof(*constraints_list), GFP_KERNEL);
	if (!constraints_list)
		return -ENOMEM;

	INIT_LIST_HEAD(&constraints_list->pzc_list);

	pzc = devm_kzalloc(priv->dev, sizeof(*pzc), GFP_KERNEL);
	if (!pzc) {
		kfree(constraints_list);
		return -ENOMEM;
	}

	pzc->id = constraint_id;
	pzc->power_zone = &spel->zone;
	pzc->ops = &constraint_ops;
	constraints_list->pzc = pzc;

	mutex_lock(&priv->spel_list_lock);
	if (!spel->constraints_list)
		spel->constraints_list = constraints_list;
	else
		list_add(&constraints_list->pzc_list, &spel->constraints_list->pzc_list);
	mutex_unlock(&priv->spel_list_lock);

	dev_dbg(priv->dev, "added constraint %d for %s\n", constraint_id, spel->name);

	return 0;
}

static int spel_register(struct spel_priv *priv, const char *name, struct spel *spel,
			 struct spel *parent)
{
	struct powercap_zone *pcz;
	struct spel *prnt = NULL;
	int ret;

	if (!priv->sh->pct)
		return -EAGAIN;

	if (!spel)
		return -EINVAL;

	if (spel->ops && !(spel->ops->get_spel_energy_uj &&
			   spel->ops->release))
		return -EINVAL;

	if (!parent || parent == priv->sh->root)
		prnt = NULL;
	else
		prnt = parent;

	pcz = powercap_register_zone(&spel->zone, priv->sh->pct, name,
				     prnt ? &prnt->zone : NULL,
				     &zone_ops, spel->nr_constraints,
				     &constraint_ops);
	if (IS_ERR(pcz))
		return PTR_ERR(pcz);

	if (parent) {
		list_add_tail(&spel->sibling, &parent->children);
		spel->parent = parent;
	} else {
		list_add_tail(&spel->sibling, &priv->sh->root->children);
		spel->parent = priv->sh->root;
	}

	for (int i = 0; i < spel->nr_constraints; i++) {
		ret = spel_add_constraint(priv, spel, i);
		if (ret) {
			dev_err(priv->dev, "Failed to add constraint %d: %d\n", i, ret);
			/* Clean up previously allocated constraints */
			spel_clean_up_constraints(priv, spel);
			/* Remove from parent's children list */
			list_del(&spel->sibling);
			/* Unregister the zone */
			powercap_unregister_zone(priv->sh->pct, &spel->zone);
			return ret;
		}
	}

	spel->enabled = true;

	dev_dbg(priv->dev, "Registered spel node '%s'\n", spel->zone.name);

	return 0;
}

static struct spel *spel_setup(struct spel_priv *priv,
			       struct spel_node *hierarchy,
			       struct spel *parent)
{
	struct spel *spel;
	int ret;

	spel = devm_kzalloc(priv->dev, sizeof(*spel), GFP_KERNEL);
	if (!spel)
		return ERR_PTR(-ENOMEM);

	strscpy(spel->name, hierarchy->name, SPEL_NAME_MAX);
	spel->np = hierarchy->np;
	spel->address = hierarchy->address;
	spel->nr_constraints = hierarchy->nr_constraints;
	spel->priv = priv;
	mutex_init(&spel->lock);
	INIT_LIST_HEAD(&spel->pz_list);
	mutex_lock(&priv->spel_list_lock);
	list_add(&spel->node, &priv->spel_list);
	mutex_unlock(&priv->spel_list_lock);
	spel_ops_init(spel, &hw_ops);
	hierarchy->spel = spel;

	ret = spel_register(priv, hierarchy->name, spel, parent);
	if (ret) {
		dev_err(priv->dev, "Failed to register spel node '%s': %d\n",
		       hierarchy->name, ret);
		spel_clean_up_constraints(priv, spel);
		mutex_lock(&priv->spel_list_lock);
		list_del(&spel->node);
		mutex_unlock(&priv->spel_list_lock);
		kfree(spel);
		return ERR_PTR(ret);
	}

	return spel;
}

static int for_spel_each_child(struct spel_priv *priv,
		struct spel_node *hierarchy,
		const struct spel_node *it, struct spel *parent)
{
	struct spel *spel = NULL;
	int i, ret;

	for (i = 0; i < priv->sh->count; i++) {
		if (hierarchy[i].parent != it)
			continue;

		spel = spel_setup(priv, &hierarchy[i], parent);
		/*
		 * A NULL pointer means there is no children, hence we
		 * continue without going deeper in the recursivity.
		 */
		if (!spel)
			continue;
		if (IS_ERR(spel)) {
			pr_warn("Failed to create '%s' in the hierarchy\n",
				hierarchy[i].name);
			return PTR_ERR(spel);
		}
		ret = for_spel_each_child(priv, hierarchy, &hierarchy[i], spel);
		if (ret)
			return ret;
	}

	return 0;
}

static int spel_pct_set_enable(struct powercap_control_type *pct, bool mode)
{
	struct spel_priv *priv = g_spel_priv;
	int ret = 0;
	struct spel *pos;

	if (!priv || priv->sh->pct != pct)
		return -EINVAL;

	list_for_each_entry(pos, &priv->spel_list, node) {
		if (pos->ops && pos->ops->set_enable) {
			ret = pos->ops->set_enable(pos, mode);
			break;
		}
	}

	if (ret)
		dev_err(priv->dev, "Failed to %s power zones\n", mode ? "enable" : "disable");

	return ret;
}

static int spel_pct_get_enable(struct powercap_control_type *pct, bool *mode)
{
	struct spel_priv *priv = g_spel_priv;
	int ret = 0;
	struct spel *pos;

	if (!priv || priv->sh->pct != pct)
		return -EINVAL;

	list_for_each_entry(pos, &priv->spel_list, node) {
		if (pos->ops && pos->ops->get_enable) {
			ret = pos->ops->get_enable(pos, mode);
			break;
		}
	}

	if (ret)
		dev_err(priv->dev, "Failed to get power zones modes\n");

	return ret;
}

static struct powercap_control_type_ops pc_ops = {
	.set_enable = spel_pct_set_enable,
	.get_enable = spel_pct_get_enable,
};

static int spel_create_root_node(struct spel_priv *priv)
{
	/*
	 * Create a root spel node and this node won't be adding
	 * into powercap sysfs. It helps to add different spel
	 * nodes as independent node under root node.
	 */
	priv->sh->root = devm_kzalloc(priv->dev, sizeof(*priv->sh->root), GFP_KERNEL);
	if (!priv->sh->root)
		return -ENOMEM;

	strscpy(priv->sh->root->name, ROOT_NODE, SPEL_NAME_MAX);
	priv->sh->root->np = NULL;
	priv->sh->root->priv = priv;
	mutex_init(&priv->sh->root->lock);
	INIT_LIST_HEAD(&priv->sh->root->pz_list);
	mutex_lock(&priv->spel_list_lock);
	list_add(&priv->sh->root->node, &priv->spel_list);
	mutex_unlock(&priv->spel_list_lock);
	spel_ops_init(priv->sh->root, NULL);

	return 0;
}

static int spel_create_hierarchy(struct spel_priv *priv, struct spel_node *hierarchy)
{
	int ret;

	if (priv->sh->pct)
		return -EBUSY;

	priv->sh->pct = powercap_register_control_type(NULL, "spel", &pc_ops);
	if (IS_ERR(priv->sh->pct)) {
		dev_err(priv->dev, "Failed to register control type\n");
		ret = PTR_ERR(priv->sh->pct);
		goto out_pct;
	}

	/* Store priv pointer in root zone's private data for callback access */
	powercap_set_zone_data(&priv->sh->root->zone, priv);

	if (!hierarchy) {
		ret = -EFAULT;
		goto out_err;
	}

	ret = for_spel_each_child(priv, hierarchy, NULL, priv->sh->root);
	if (ret)
		goto out_err;

	return 0;

out_err:
	powercap_unregister_control_type(priv->sh->pct);
out_pct:
	priv->sh->pct = NULL;

	return ret;
}

static void __spel_destroy_hierarchy(struct spel_priv *priv, struct spel *spel)
{
	struct spel *child, *aux;

	list_for_each_entry_safe(child, aux, &spel->children, sibling)
		__spel_destroy_hierarchy(priv, child);

	/*
	 * At this point, we know all children were removed from the
	 * recursive call before
	 */
	if (spel != priv->sh->root) {
		spel_unregister(priv, spel);
	} else {
		mutex_lock(&priv->spel_list_lock);
		list_del(&spel->node);
		mutex_unlock(&priv->spel_list_lock);
		spel_clean_up_constraints(priv, spel);
		kfree(spel);
	}
}

static void spel_destroy_hierarchy(struct spel_priv *priv)
{
	if (!priv->sh->pct)
		return;

	__spel_destroy_hierarchy(priv, priv->sh->root);

	powercap_unregister_control_type(priv->sh->pct);

	priv->sh->pct = NULL;

	priv->sh->root = NULL;
}

static int of_each_spel_child(struct spel_priv *priv)
{
	struct spel_node *sys = NULL;
	struct spel_node *soc = NULL;

	for (int i = 0; i < priv->sh->count; ++i) {
		strscpy(priv->sh->hierarchy[priv->sh->cur_index].name,
			children_nodes[i].name, SPEL_NAME_MAX);
		priv->sh->hierarchy[priv->sh->cur_index].address = MMIO_OFFSET(
			priv->nodes, children_nodes[i].offset);
		priv->sh->hierarchy[priv->sh->cur_index].nr_constraints =
			children_nodes[i].nr_constraints;
		switch (children_nodes[i].index) {
		case SYS:
			sys = &priv->sh->hierarchy[priv->sh->cur_index];
			priv->sh->hierarchy[priv->sh->cur_index].parent = NULL;
			break;
		case SOC:
			soc = &priv->sh->hierarchy[priv->sh->cur_index];
			priv->sh->hierarchy[priv->sh->cur_index].parent = sys;
			break;
		case SOC_CL0:
		case SOC_CL1:
		case SOC_CL2:
		case SOC_IGPU:
		case SOC_DGPU:
		case SOC_NSP:
		case SOC_MMCX:
		case SOC_INFRA:
		case SOC_DRAM:
		case SOC_MDM:
		case SOC_WLAN:
		case SOC_USB1:
		case SOC_USB2:
		case SOC_USB3:
			priv->sh->hierarchy[priv->sh->cur_index].parent = soc;
			break;
		}
		priv->sh->cur_index++;
	}

	return 0;
}

static int spel_get_mem_resource(struct platform_device *pdev, struct spel_priv *priv)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem **dest;

	for (int i = 0; i < 3; ++i) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(priv->dev, "Cannot get MEM resource #%d\n", i);
			return -EINVAL;
		}

		dev_dbg(priv->dev, "mem@0x%llx size:%lld\n", res->start,
				resource_size(res));

		switch (i) {
		case 0:
			dest = &priv->nodes;
			break;
		case 1:
			dest = &priv->constraints;
			break;
		case 2:
			dest = &priv->config;
			break;
		default:
			dev_err(priv->dev, "Invalid resource index: %d\n", i);
			return -EINVAL;
		}

		*dest = devm_ioremap_resource(dev, res);

		if (IS_ERR(*dest)) {
			dev_err(priv->dev, "Cannot get regmap for resource #%d\n", i);
			return PTR_ERR(*dest);
		}
	}

	return 0;
}

static int spel_device_probe(struct platform_device *pdev)
{
	struct spel_priv *priv;
	int ret;

	/* Allocate driver private data using devm_kzalloc */
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Initialize list and mutex */
	INIT_LIST_HEAD(&priv->spel_list);
	mutex_init(&priv->spel_list_lock);

	priv->dev = &pdev->dev;

	/* Store driver data in platform device */
	platform_set_drvdata(pdev, priv);

	ret = spel_get_mem_resource(pdev, priv);
	if (ret)
		return ret;

	ret = spel_get_units(priv, (void __iomem *)priv->config);
	if (ret)
		return ret;

	spel_init_constraints((void __iomem *)priv->config);

	priv->sh = devm_kzalloc(&pdev->dev, sizeof(*priv->sh), GFP_KERNEL);
	if (!priv->sh)
		return -ENOMEM;

	ret = get_spel_count();
	if (ret <= 1) {
		dev_err(priv->dev, "Invalid count: %d\n", ret);
		return -ENODEV;
	}

	priv->sh->count = ret;
	priv->sh->hierarchy = devm_kcalloc(&pdev->dev, priv->sh->count,
					   sizeof(*priv->sh->hierarchy), GFP_KERNEL);
	if (!priv->sh->hierarchy)
		return -ENOMEM;

	ret = spel_create_root_node(priv);
	if (ret < 0)
		return ret;

	ret = of_each_spel_child(priv);
	if (ret) {
		dev_err(priv->dev, "Failed to read powerzones hierarchy: %d\n", ret);
		goto release_root;
	}

	ret = spel_create_hierarchy(priv, priv->sh->hierarchy);
	if (ret < 0)
		goto release_root;

	/* Set global pointer for control type callbacks */
	g_spel_priv = priv;

	return 0;

release_root:
	mutex_lock(&priv->spel_list_lock);
	list_del(&priv->sh->root->node);
	mutex_unlock(&priv->spel_list_lock);
	kfree(priv->sh->root);

	return ret;
}

static void spel_device_remove(struct platform_device *pdev)
{
	struct spel_priv *priv = platform_get_drvdata(pdev);

	if (!priv || !priv->sh)
		return;

	/* Clear global pointer */
	g_spel_priv = NULL;

	spel_destroy_hierarchy(priv);
}

static struct resource spel_resources[] = {
	{
		.start = 0x0ef3e000,
		.end   = 0x0ef3efff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0ef3d000,
		.end   = 0x0ef3dfff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0ef3b000,
		.end   = 0x0ef3bfff,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device spel_pdev = {
	.name          = SPEL_HW,
	.id            = -1,
	.num_resources = ARRAY_SIZE(spel_resources),
	.resource      = spel_resources,
};

static struct platform_driver qti_spel_module = {
	.probe          = spel_device_probe,
	.remove         = spel_device_remove,
	.driver         = {
		.name   = SPEL_HW,
	},
};

static int __init qti_spel_init(void)
{
	int err;

	err = platform_driver_register(&qti_spel_module);
	if (err)
		return err;

	err = platform_device_register(&spel_pdev);
	if (err < 0)
		platform_driver_unregister(&qti_spel_module);


	return err;
}

static void __exit qti_spel_exit(void)
{
	platform_device_unregister(&spel_pdev);
	platform_driver_unregister(&qti_spel_module);
}

module_init(qti_spel_init);
module_exit(qti_spel_exit);

MODULE_DESCRIPTION("QTI Power Telemetry interface driver");
MODULE_LICENSE("GPL");
